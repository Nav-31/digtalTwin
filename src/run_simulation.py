# run_simulation.py
import os
import sys
import time
import subprocess
import traci
import random
import csv
import math
import logging #+

#+ Import the setup function and the camera components
from logging_config import setup_logging #+
from camera import update_virtual_cameras, close_camera_log, CAMERA_CACHE, CAMERAS #+

#+ Get a logger instance for this module
logger = logging.getLogger(__name__)

# Ensure SUMO tools are in path
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")

def handle_between_update(connB, vid, state, vehicle_last_speed):
    """
    Called when we’re *not* applying an explicit mirror update.
    This handles smooth speed adjustments between sensor updates.
    """
    try:
        is_near_light, dist_to_light, light_state = detect_traffic_light_proximity(connB, vid)
        if is_vehicle_stopped_at_light(connB, vid):
            logger.debug(f"{vid} is stopped at a light; letting SUMO control.")
            return
        if is_near_light and dist_to_light < HybridConfig.CRITICAL_LIGHT_DISTANCE:
            logger.debug(f"{vid} is near a light; letting SUMO control.")
            return

        # Safe area: smooth speed
        if state['is_stopped']:
            cur = connB.vehicle.getSpeed(vid)
            new_speed = cur * HybridConfig.GRADUAL_STOP_FACTOR if cur > 0.1 else 0
            connB.vehicle.setSpeed(vid, new_speed)
            logger.debug(f"{vid} is stopping gradually. Speed -> {new_speed:.2f}")
        else:
            last = vehicle_last_speed.get(vid, state['speed'])
            smooth = HybridConfig.SPEED_SMOOTH_FACTOR * state['speed'] + (1 - HybridConfig.SPEED_SMOOTH_FACTOR) * last
            connB.vehicle.setSpeed(vid, smooth)
            vehicle_last_speed[vid] = smooth
            logger.debug(f"{vid} speed smoothed to {smooth:.2f}")
    except traci.TraCIException as e:
        logger.warning(f"Error during 'between update' for {vid}: {e}")
    except Exception as e:
        logger.error(f"Unexpected error in handle_between_update for {vid}: {e}", exc_info=True)


# Configuration parameters
class HybridConfig:
    TRAFFIC_LIGHT_PROXIMITY = 30
    CRITICAL_LIGHT_DISTANCE = 15
    STOP_LINE_DISTANCE = 20
    MIN_GPS_ERROR_FOR_CORRECTION = 5.0
    MAX_GPS_ERROR = 10.0
    MIN_CORRECTION_FACTOR = 0.1
    MAX_CORRECTION_FACTOR = 0.8
    NEAR_LIGHT_CORRECTION_FACTOR = 0.4
    SPEED_THRESHOLD_STOPPED = 0.5
    SPEED_SMOOTH_FACTOR = 0.7
    GRADUAL_STOP_FACTOR = 0.7
    #-- STATUS_LOG_INTERVAL is no longer needed, logger handles timestamps
    # STATUS_LOG_INTERVAL = 50

# --- Mirroring Strategies ---
class BaseMirroringStrategy:
    def mirror(self, connA, connB, vid, step, state):
        raise NotImplementedError

class GPSMirroringStrategy(BaseMirroringStrategy):
    def mirror(self, connA, connB, vid, step, state):
        return hybrid_gps_mirroring(connA, connB, vid, state['gps_pos'], state['speed'], state['is_stopped'])

class CameraMirroringStrategy(BaseMirroringStrategy):
    def mirror(self, connA, connB, vid, step, state):
        det = state['camera_cache'].get(vid)
        if det:
            det_step, cam_id, speed, lane, angle, noisy_dist = det
            if det_step >= state['last_mirror_step']: # Use ">=" to include current step
                logger.info(f"Using fresh camera detection for {vid} from {cam_id} (detected at step {det_step}).")
                cam = CAMERAS[cam_id]
                theta = math.radians(90 - angle) # SUMO angle (0=N, 90=E) to standard angle (0=E, 90=N)
                x = cam['x'] + noisy_dist * math.cos(theta)
                y = cam['y'] + noisy_dist * math.sin(theta)
                
                connB.vehicle.moveToXY(
                    vid, edgeID=state['road_id'], laneIndex=state['lane_index'],
                    x=x, y=y, keepRoute=1
                )
                connB.vehicle.setSpeed(vid, speed)
                logger.info(f"CAMERA_UPDATE: {vid} moved to ({x:.1f}, {y:.1f}) with speed {speed:.1f}.")
                return True, 'camera_update'
        logger.debug(f"No fresh camera detection for {vid} at step {step}.")
        return False, 'camera_no_update' # Return False as no action was taken

class LoopMirroringStrategy(BaseMirroringStrategy):
    def mirror(self, connA, connB, vid, step, state):
        return False, 'loop_not_implemented'

class RandomMirroringStrategy(BaseMirroringStrategy):
    def mirror(self, connA, connB, vid, step, state):
        logger.warning(f"All sensors failed for {vid}. Using random fallback (teleport to true position).")
        true_x, true_y = state['true_pos']
        connB.vehicle.moveToXY(vid, edgeID=state['road_id'], laneIndex=state['lane_index'], x=true_x, y=true_y, keepRoute=1)
        return True, 'random_fallback'

# Sequential fallback executor
def mirror_with_fallback(connA, connB, vid, step, state):
    # Try GPS
    if state.get('gps_pos'):
        ok, mode = GPSMirroringStrategy().mirror(connA, connB, vid, step, state)
        if ok: return ok, mode
    # Try camera
    ok, mode = CameraMirroringStrategy().mirror(connA, connB, vid, step, state)
    if ok: return ok, mode
    # Try induction loops
    ok, mode = LoopMirroringStrategy().mirror(connA, connB, vid, step, state)
    if ok: return ok, mode
    # Finally random
    return RandomMirroringStrategy().mirror(connA, connB, vid, step, state)


# --- Utility Functions ---
def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def get_noisy_gps_position(conn, vehicle_id, std_dev=2.5):
    try:
        true_pos = conn.vehicle.getPosition(vehicle_id)
        # In the original code, this was returning None. Assuming you want noisy GPS.
        noisy_x = true_pos[0] + random.gauss(0, std_dev)
        noisy_y = true_pos[1] + random.gauss(0, std_dev)
        return noisy_x, noisy_y, true_pos
    except Exception as e:
        logger.error(f"Failed to get GPS position for {vehicle_id}: {e}")
        return None, None, (None, None)

# ... (detect_traffic_light_proximity, is_vehicle_stopped_at_light, calculate_gps_correction_factor, get_vehicle_state remain mostly the same, but without prints)
def detect_traffic_light_proximity(conn, vehicle_id, proximity_threshold=None):
    if proximity_threshold is None: proximity_threshold = HybridConfig.TRAFFIC_LIGHT_PROXIMITY
    try:
        next_tls = conn.vehicle.getNextTLS(vehicle_id)
        if not next_tls: return False, float('inf'), None
        tl_id, tl_link_index, distance, state = next_tls[0]
        return distance <= proximity_threshold, distance, state
    except Exception:
        try:
            road_id = conn.vehicle.getRoadID(vehicle_id)
            if road_id.startswith(':'): return True, 0, 'r'
        except Exception: pass
        return False, float('inf'), None

def is_vehicle_stopped_at_light(conn, vehicle_id, speed_threshold=None):
    if speed_threshold is None: speed_threshold = HybridConfig.SPEED_THRESHOLD_STOPPED
    try:
        speed = conn.vehicle.getSpeed(vehicle_id)
        is_near_light, distance, light_state = detect_traffic_light_proximity(conn, vehicle_id)
        if (speed < speed_threshold and 
            is_near_light and 
            distance < HybridConfig.STOP_LINE_DISTANCE and 
            light_state and light_state.lower() in ['r', 'y']):
            return True
        return False
    except Exception: return False

def calculate_gps_correction_factor(gps_error, max_error=None, min_factor=None, max_factor=None):
    if max_error is None: max_error = HybridConfig.MAX_GPS_ERROR
    if min_factor is None: min_factor = HybridConfig.MIN_CORRECTION_FACTOR
    if max_factor is None: max_factor = HybridConfig.MAX_CORRECTION_FACTOR
    if gps_error <= 1.0: return 0.0
    normalized_error = min(gps_error / max_error, 1.0)
    return min_factor + (max_factor - min_factor) * normalized_error

def get_vehicle_state(conn, vehicle_id, stop_counter, speed_threshold=0.3):
    try:
        speed = conn.vehicle.getSpeed(vehicle_id)
        if speed < speed_threshold:
            stop_counter[vehicle_id] = stop_counter.get(vehicle_id, 0) + 1
        else:
            stop_counter[vehicle_id] = 0
        return speed, stop_counter.get(vehicle_id, 0) >= 2
    except traci.TraCIException: return 0, True


def hybrid_gps_mirroring(connA, connB, vehicle_id, gps_pos, current_speed, is_stopped):
    """MAIN HYBRID FUNCTION: Combines GPS positioning with traffic rule compliance."""
    try:
        current_mirror_pos = connB.vehicle.getPosition(vehicle_id)
        current_mirror_speed = connB.vehicle.getSpeed(vehicle_id)
        gps_error = euclidean(gps_pos, current_mirror_pos)
        is_near_light_mirror, dist_mirror, state_mirror = detect_traffic_light_proximity(connB, vehicle_id)
        
        intervention_needed = False
        position_intervention = "none"

        if gps_error > HybridConfig.MIN_GPS_ERROR_FOR_CORRECTION:
            if is_vehicle_stopped_at_light(connB, vehicle_id):
                position_intervention = "no_move_at_light"
                logger.info(f"GPS: {vehicle_id} has error {gps_error:.1f}m, but is stopped at a light. No position change.")
            elif is_near_light_mirror and dist_mirror < HybridConfig.CRITICAL_LIGHT_DISTANCE:
                position_intervention = "minimal_correction"
                intervention_needed = True
                logger.info(f"GPS: {vehicle_id} is near a light. Applying minimal correction for error {gps_error:.1f}m.")
            else:
                position_intervention = "gps_correction"
                intervention_needed = True
                logger.info(f"GPS: Applying correction for {vehicle_id} with error {gps_error:.1f}m.")

        if intervention_needed:
            road_id = connA.vehicle.getRoadID(vehicle_id)
            lane_id = connA.vehicle.getLaneID(vehicle_id)
            lane_index = int(lane_id.split('_')[-1]) if '_' in lane_id else 0
            
            correction_factor = calculate_gps_correction_factor(gps_error)
            if position_intervention == "minimal_correction":
                correction_factor = min(correction_factor, HybridConfig.NEAR_LIGHT_CORRECTION_FACTOR)
            
            corrected_x = current_mirror_pos[0] + correction_factor * (gps_pos[0] - current_mirror_pos[0])
            corrected_y = current_mirror_pos[1] + correction_factor * (gps_pos[1] - current_mirror_pos[1])
            
            connB.vehicle.moveToXY(vehicle_id, edgeID=road_id, laneIndex=lane_index, x=corrected_x, y=corrected_y, keepRoute=1)

        # Speed control
        if is_vehicle_stopped_at_light(connB, vehicle_id) or (is_near_light_mirror and dist_mirror < HybridConfig.CRITICAL_LIGHT_DISTANCE):
            logger.debug(f"SPEED: Letting SUMO control speed for {vehicle_id} near traffic light.")
        else:
            if is_stopped:
                new_speed = current_mirror_speed * HybridConfig.GRADUAL_STOP_FACTOR if current_mirror_speed > 0.1 else 0
                connB.vehicle.setSpeed(vehicle_id, new_speed)
            else:
                smooth_speed = HybridConfig.SPEED_SMOOTH_FACTOR * current_speed + (1 - HybridConfig.SPEED_SMOOTH_FACTOR) * current_mirror_speed
                connB.vehicle.setSpeed(vehicle_id, smooth_speed)
        
        return True, position_intervention
        
    except traci.TraCIException as e:
        logger.warning(f"Hybrid mirroring failed for {vehicle_id}: {e}")
        return False, "error"
    except Exception as e:
        logger.error(f"Unexpected error in hybrid_gps_mirroring for {vehicle_id}: {e}", exc_info=True)
        return False, "error"


# Main mirroring loop
def mirror_simulation(configA, configB, portA, portB, max_steps, step_length):
    try:
        sumo_processA = subprocess.Popen(["sumo-gui", "-c", configA, "--start", "--remote-port", str(portA)])
        sumo_processB = subprocess.Popen(["sumo-gui", "-c", configB, "--start", "--remote-port", str(portB)])
        logger.info("Waiting for SUMO GUIs to launch...")
        time.sleep(5)

        connA = traci.connect(port=portA)
        connB = traci.connect(port=portB)
        logger.info("Successfully connected to both SUMO simulations.")
    except Exception as e:
        logger.critical(f"Failed to start or connect to SUMO: {e}", exc_info=True)
        return

    step = 0
    seen_vehicles = set()
    just_added_steps = {}
    last_mirroring = {}
    gps_update_interval = 10 # steps
    stop_counter = {}
    vehicle_last_speed = {}

    gps_log_file = open("gps_vs_true_log.csv", "w", newline='')
    gps_logger = csv.writer(gps_log_file)
    gps_logger.writerow(["step","bus_id","true_x","true_y","noisy_x","noisy_y","simb_x","simb_y","lag_distance","intervention_type"])

    try:
        while step < max_steps:
            connA.simulationStep()
            update_virtual_cameras(step, connA)
            connB.simulationStep()

            # Synchronize Traffic Lights
            for tl_id in connA.trafficlight.getIDList():
                try:
                    state = connA.trafficlight.getRedYellowGreenState(tl_id)
                    connB.trafficlight.setRedYellowGreenState(tl_id, state)
                except traci.TraCIException:
                    logger.warning(f"Could not sync traffic light {tl_id}. It may not exist in Sim B.")

            # Process all vehicles from the 'real' simulation
            for vid in connA.vehicle.getIDList():
                if vid not in seen_vehicles:
                    try:
                        pos = connA.vehicle.getPosition(vid)
                        speed = connA.vehicle.getSpeed(vid)
                        road_id = connA.vehicle.getRoadID(vid)
                        route_edges = connA.vehicle.getRoute(vid)
                        route_id = f"{vid}_route"
                        veh_type = connA.vehicle.getTypeID(vid)
                        
                        connB.route.add(route_id, route_edges)
                        connB.vehicle.add(vid, routeID=route_id, typeID=veh_type)
                        connB.vehicle.moveToXY(vid, edgeID=road_id, laneIndex=0, x=pos[0], y=pos[1], keepRoute=1)
                        connB.vehicle.setSpeed(vid, speed)
                        connB.vehicle.setColor(vid, (0, 255, 0, 255)) # Green for mirrored vehicle

                        seen_vehicles.add(vid)
                        just_added_steps[vid] = step
                        logger.info(f"SPAWN: Mirrored new vehicle {vid} in Sim B.")
                    except traci.TraCIException as e:
                        logger.error(f"Failed to add vehicle {vid} to Sim B: {e}")
                        continue
                
                # Skip updates for a few steps after adding to stabilize
                if step - just_added_steps.get(vid,0) < 2:
                    continue

                # Get state from the 'real' vehicle
                true_pos = connA.vehicle.getPosition(vid)
                speed, is_stopped = get_vehicle_state(connA, vid, stop_counter)
                road_id = connA.vehicle.getRoadID(vid)
                lane_id = connA.vehicle.getLaneID(vid)
                lane_index = int(lane_id.split('_')[-1]) if lane_id and '_' in lane_id else 0

                state = {
                    'speed': speed, 'is_stopped': is_stopped, 'true_pos': true_pos,
                    'road_id': road_id, 'lane_index': lane_index,
                    'last_mirror_step': last_mirroring.get(vid, 0),
                    'camera_cache': CAMERA_CACHE, 'gps_pos': None
                }

                # Decide if a sensor update (mirroring) is due
                is_mirroring_due = (step - last_mirroring.get(vid, -gps_update_interval) >= gps_update_interval)
                if is_mirroring_due:
                    last_mirroring[vid] = step
                    
                    gps_x, gps_y, _ = get_noisy_gps_position(connA, vid)
                    if gps_x is not None:
                        state['gps_pos'] = (gps_x, gps_y)
                    
                    logger.debug(f"Attempting sensor fusion mirror for {vid} at step {step}.")
                    success, intervention_type = mirror_with_fallback(connA, connB, vid, step, state)

                    # Log data for analysis
                    updated_pos = connB.vehicle.getPosition(vid)
                    lag = euclidean(true_pos, updated_pos)
                    gps_logger.writerow([step, vid, true_pos[0], true_pos[1], gps_x, gps_y, updated_pos[0], updated_pos[1], lag, intervention_type])
                
                else: # Between sensor updates
                    handle_between_update(connB, vid, state, vehicle_last_speed)
            
            step += 1
            time.sleep(step_length)

    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user.")
    except Exception as e:
        logger.critical(f"A critical error occurred in the main loop: {e}", exc_info=True)
    finally:
        logger.info("Cleaning up and closing connections...")
        # The 'in locals()' check is sufficient to prevent errors if connection failed.
        # traci.close() is safe to call without an isconnected() check.
        if 'connA' in locals():
            connA.close()
        if 'connB' in locals():
            connB.close()
        
        if 'gps_log_file' in locals() and not gps_log_file.closed:
            gps_log_file.close()
            
        close_camera_log()
        
        if 'sumo_processA' in locals():
            sumo_processA.terminate()
        if 'sumo_processB' in locals():
            sumo_processB.terminate()
            


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Enhanced Digital Twin Urban Traffic Mirror with Modular Fallback")
    parser.add_argument("--configA", required=True, help="Path to SUMO config for Simulation A (real)")
    parser.add_argument("--configB", required=True, help="Path to SUMO config for Simulation B (mirror)")
    parser.add_argument("--portA", type=int, default=8813)
    parser.add_argument("--portB", type=int, default=8814)
    parser.add_argument("--steps", type=int, default=10000)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--verbose", action="store_true", help="Show detailed DEBUG messages on console.")
    args = parser.parse_args()

    #+ Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    setup_logging(console_level=log_level)

    mirror_simulation(args.configA, args.configB, args.portA, args.portB, args.steps, args.interval)

if __name__ == "__main__":
    main()