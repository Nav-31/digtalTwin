import os
import sys
import time
import subprocess
import traci
import random
import csv
import math
from camera import update_virtual_cameras, close_camera_log


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")

# Configuration parameters for hybrid mirroring
class HybridConfig:
    """Configuration parameters for hybrid mirroring"""
    
    # Traffic light detection
    TRAFFIC_LIGHT_PROXIMITY = 30  # meters
    CRITICAL_LIGHT_DISTANCE = 15  # meters - very close to light
    STOP_LINE_DISTANCE = 20      # meters - consider stopped at light
    
    # GPS correction thresholds
    MIN_GPS_ERROR_FOR_CORRECTION = 5.0   # meters
    MAX_GPS_ERROR = 10.0                 # meters
    MIN_CORRECTION_FACTOR = 0.1          # minimum correction strength
    MAX_CORRECTION_FACTOR = 0.8          # maximum correction strength
    NEAR_LIGHT_CORRECTION_FACTOR = 0.4   # correction when near lights
    
    # Speed control
    SPEED_THRESHOLD_STOPPED = 0.5        # m/s - consider vehicle stopped
    SPEED_SMOOTH_FACTOR = 0.7            # smoothing for speed changes
    GRADUAL_STOP_FACTOR = 0.7            # deceleration rate
    
    # Logging
    STATUS_LOG_INTERVAL = 50             # steps between status logs

# Initialize logging
gps_log_file = open("gps_vs_true_log.csv", "w", newline='')
gps_logger = csv.writer(gps_log_file)
gps_logger.writerow(["step", "bus_id", "true_x", "true_y", "noisy_x", "noisy_y", "simb_x", "simb_y", "lag_distance", "intervention_type"])

def get_noisy_gps_position(conn, vehicle_id, std_dev=2.5):
    """Reduced noise for more stable positioning"""
    try:
        true_pos = conn.vehicle.getPosition(vehicle_id)
        noisy_x = true_pos[0] + random.gauss(0, std_dev)
        noisy_y = true_pos[1] + random.gauss(0, std_dev)
        return noisy_x, noisy_y, true_pos
    except Exception as e:
        print(f"[ERROR] Failed to get GPS position for {vehicle_id}: {e}")
        return None, None, (None, None)

def euclidean(a, b):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def detect_traffic_light_proximity(conn, vehicle_id, proximity_threshold=None):
    """
    Detect if vehicle is approaching or at a traffic light
    
    Args:
        conn: TRACI connection
        vehicle_id: Vehicle ID to check
        proximity_threshold: Distance threshold in meters
    
    Returns:
        tuple: (is_near_traffic_light, distance_to_light, light_state)
    """
    if proximity_threshold is None:
        proximity_threshold = HybridConfig.TRAFFIC_LIGHT_PROXIMITY
        
    try:
        # Get next traffic light information
        next_tls = conn.vehicle.getNextTLS(vehicle_id)
        
        if not next_tls:
            return False, float('inf'), None
        
        # next_tls returns list of (tl_id, tl_link_index, distance, state)
        tl_id, tl_link_index, distance, state = next_tls[0]
        
        # Check if vehicle is close to traffic light
        is_near = distance <= proximity_threshold
        
        return is_near, distance, state
        
    except Exception as e:
        # Fallback: check if vehicle is on junction
        try:
            road_id = conn.vehicle.getRoadID(vehicle_id)
            if road_id.startswith(':'):  # Junction internal edge
                return True, 0, 'r'  # Assume red for safety
        except:
            pass
        return False, float('inf'), None

def calculate_gps_correction_factor(gps_error, max_error=None, min_factor=None, max_factor=None):
    """
    Calculate how much GPS correction to apply based on error magnitude
    
    Args:
        gps_error: Distance between GPS and current position
        max_error: Maximum error before applying full correction
        min_factor: Minimum correction factor (for small errors)
        max_factor: Maximum correction factor (for large errors)
    
    Returns:
        float: Correction factor between min_factor and max_factor
    """
    if max_error is None:
        max_error = HybridConfig.MAX_GPS_ERROR
    if min_factor is None:
        min_factor = HybridConfig.MIN_CORRECTION_FACTOR
    if max_factor is None:
        max_factor = HybridConfig.MAX_CORRECTION_FACTOR
        
    if gps_error <= 1.0:
        return 0.0  # No correction needed for small errors
    
    # Linear interpolation between min and max factors
    normalized_error = min(gps_error / max_error, 1.0)
    correction_factor = min_factor + (max_factor - min_factor) * normalized_error
    
    return correction_factor

def is_vehicle_stopped_at_light(conn, vehicle_id, speed_threshold=None):
    """
    Determine if vehicle is stopped specifically due to traffic light
    
    Args:
        conn: TRACI connection
        vehicle_id: Vehicle ID to check
        speed_threshold: Speed below which vehicle is considered stopped
    
    Returns:
        bool: True if stopped at traffic light
    """
    if speed_threshold is None:
        speed_threshold = HybridConfig.SPEED_THRESHOLD_STOPPED
        
    try:
        speed = conn.vehicle.getSpeed(vehicle_id)
        is_near_light, distance, light_state = detect_traffic_light_proximity(conn, vehicle_id)
        
        # Vehicle is stopped at light if:
        # 1. Speed is very low
        # 2. Close to traffic light (< STOP_LINE_DISTANCE)
        # 3. Traffic light is red or yellow
        if (speed < speed_threshold and 
            is_near_light and 
            distance < HybridConfig.STOP_LINE_DISTANCE and 
            light_state and light_state.lower() in ['r', 'y']):
            return True
            
        return False
        
    except Exception as e:
        return False

def hybrid_gps_mirroring(connA, connB, vehicle_id, gps_pos, current_speed, is_stopped, step):
    """
    MAIN HYBRID FUNCTION: Combines GPS positioning with traffic rule compliance
    
    This is the core of Solution 3 - it replaces original GPS processing logic
    
    Args:
        connA: Connection to real simulation (source of truth)
        connB: Connection to mirror simulation (where we apply corrections)
        vehicle_id: Vehicle to update
        gps_pos: GPS coordinates (x, y) with noise
        current_speed: Current speed from real simulation
        is_stopped: Whether vehicle is stopped in real simulation
        step: Current simulation step
    
    Returns:
        tuple: (success, intervention_type)
    """
    intervention_type = "none"
    
    try:
        # STEP 1: Get current state of mirrored vehicle
        current_mirror_pos = connB.vehicle.getPosition(vehicle_id)
        current_mirror_speed = connB.vehicle.getSpeed(vehicle_id)
        
        # STEP 2: Calculate GPS positioning error
        gps_error = euclidean(gps_pos, current_mirror_pos)
        
        # STEP 3: Check traffic light status in mirror simulation
        is_near_light_mirror, dist_mirror, state_mirror = detect_traffic_light_proximity(connB, vehicle_id)
        
        # STEP 4: Determine intervention strategy
        intervention_needed = False
        
        if gps_error > HybridConfig.MIN_GPS_ERROR_FOR_CORRECTION:  # Significant GPS drift
            if is_near_light_mirror and dist_mirror < HybridConfig.CRITICAL_LIGHT_DISTANCE:
                # Very close to traffic light - minimal intervention
                intervention_type = "minimal_correction"
                intervention_needed = True
            elif is_vehicle_stopped_at_light(connB, vehicle_id):
                # Vehicle stopped at light - don't move it
                intervention_type = "no_position_change"
                intervention_needed = False  # No position change
            else:
                # Safe to apply GPS correction
                intervention_type = "gps_correction"
                intervention_needed = True
        
        # STEP 5: Apply appropriate intervention
        if intervention_needed:
            road_id = connA.vehicle.getRoadID(vehicle_id)
            
            try:
                lane_id = connA.vehicle.getLaneID(vehicle_id)
                lane_index = int(lane_id.split('_')[-1]) if '_' in lane_id else 0
            except:
                lane_index = 0
            
            if intervention_type == "gps_correction":
                # Full GPS correction - safe area
                correction_factor = calculate_gps_correction_factor(gps_error)
                
                corrected_x = current_mirror_pos[0] + correction_factor * (gps_pos[0] - current_mirror_pos[0])
                corrected_y = current_mirror_pos[1] + correction_factor * (gps_pos[1] - current_mirror_pos[1])
                
                # Apply position update
                connB.vehicle.moveToXY(vehicle_id, edgeID=road_id, laneIndex=lane_index,
                                     x=corrected_x, y=corrected_y, keepRoute=1)
                
                if step % HybridConfig.STATUS_LOG_INTERVAL == 0:
                    print(f"[GPS_CORRECT] {vehicle_id}: error={gps_error:.1f}m, factor={correction_factor:.2f}")
                
            elif intervention_type == "minimal_correction":
                # Near traffic light - very small correction only
                correction_factor = HybridConfig.NEAR_LIGHT_CORRECTION_FACTOR
                
                corrected_x = current_mirror_pos[0] + correction_factor * (gps_pos[0] - current_mirror_pos[0])
                corrected_y = current_mirror_pos[1] + correction_factor * (gps_pos[1] - current_mirror_pos[1])
                
                connB.vehicle.moveToXY(vehicle_id, edgeID=road_id, laneIndex=lane_index,
                                     x=corrected_x, y=corrected_y, keepRoute=1)
                
                if step % HybridConfig.STATUS_LOG_INTERVAL == 0:
                    print(f"[MINIMAL_CORRECT] {vehicle_id}: near light, gentle correction")
        
        elif intervention_type == "no_position_change":
            # Don't change position - vehicle properly stopped at light
            if step % HybridConfig.STATUS_LOG_INTERVAL == 0:
                print(f"[NO_MOVE] {vehicle_id}: stopped at traffic light")
        
        # STEP 6: Handle speed control intelligently
        speed_control_method = "gps"
        
        if is_vehicle_stopped_at_light(connB, vehicle_id):
            # Vehicle is stopped at red light - let SUMO handle it
            speed_control_method = "sumo"
            if step % HybridConfig.STATUS_LOG_INTERVAL == 0:
                print(f"[SPEED_SUMO] {vehicle_id}: letting SUMO control speed at traffic light")
            
        elif is_near_light_mirror and dist_mirror < HybridConfig.CRITICAL_LIGHT_DISTANCE:
            # Very close to light - let SUMO's traffic logic handle speed
            speed_control_method = "sumo"
            if step % HybridConfig.STATUS_LOG_INTERVAL == 0:
                print(f"[SPEED_SUMO] {vehicle_id}: close to light, SUMO controls speed")
            
        else:
            # Safe to apply GPS-based speed control
            if is_stopped:
                # Real vehicle is stopped - gradually stop mirror vehicle
                if current_mirror_speed > 0.1:
                    connB.vehicle.setSpeed(vehicle_id, current_mirror_speed * HybridConfig.GRADUAL_STOP_FACTOR)
                else:
                    connB.vehicle.setSpeed(vehicle_id, 0)
                if step % HybridConfig.STATUS_LOG_INTERVAL == 0:
                    print(f"[SPEED_GPS] {vehicle_id}: stopping (real vehicle stopped)")
            else:
                # Match real vehicle speed with smoothing
                target_speed = current_speed
                smooth_speed = HybridConfig.SPEED_SMOOTH_FACTOR * target_speed + (1 - HybridConfig.SPEED_SMOOTH_FACTOR) * current_mirror_speed
                connB.vehicle.setSpeed(vehicle_id, smooth_speed)
                if step % HybridConfig.STATUS_LOG_INTERVAL == 0:
                    print(f"[SPEED_GPS] {vehicle_id}: matching real speed {target_speed:.1f}")
        
        # STEP 7: Enhanced logging
        if step % (HybridConfig.STATUS_LOG_INTERVAL * 2) == 0:  # Less frequent detailed status
            print(f"[STATUS] {vehicle_id}:")
            print(f"  GPS Error: {gps_error:.1f}m")
            print(f"  Near Light: {is_near_light_mirror} (dist: {dist_mirror:.1f}m)")
            print(f"  Intervention: {intervention_type}")
            print(f"  Speed Control: {speed_control_method}")
        
        return True, intervention_type
        
    except Exception as e:
        print(f"[ERROR] Hybrid mirroring failed for {vehicle_id}: {e}")
        return False, "error"

def get_vehicle_state(conn, vehicle_id, stop_counter, speed_threshold=0.3):
    """
    Determine if vehicle is truly stopped or just slow
    """
    try:
        speed = conn.vehicle.getSpeed(vehicle_id)
        
        # Update stop counter
        if speed < speed_threshold:
            stop_counter[vehicle_id] = stop_counter.get(vehicle_id, 0) + 1
        else:
            stop_counter[vehicle_id] = 0
        
        # Vehicle is considered stopped after 2 consecutive slow readings
        is_stopped = stop_counter.get(vehicle_id, 0) >= 2
        
        return speed, is_stopped
    except:
        return 0, True

def mirror_simulation(configA, configB, portA=8813, portB=8814, max_steps=10000, step_length=0.1):
    sumo_processA = subprocess.Popen(["sumo-gui", "-c", configA, "--start", "--remote-port", str(portA)])
    sumo_processB = subprocess.Popen(["sumo-gui", "-c", configB, "--start", "--remote-port", str(portB)])
    time.sleep(2)

    connA = traci.connect(port=portA)
    connB = traci.connect(port=portB)
    print("Connected to both simulations.")

    step = 0
    seen_vehicles = set()
    just_added_steps = {}
    last_gps_update = {}
    gps_update_interval = 10  # GPS update frequency
    stop_counter = {}
    
    # Enhanced state tracking
    vehicle_last_speed = {}

    try:
        while step < max_steps:
            connA.simulationStep()
            update_virtual_cameras(step, connA)
            connB.simulationStep()

            current_vehicles = connA.vehicle.getIDList()
            
            for vid in current_vehicles:
                try:
                    # Get vehicle info from SimA
                    pos = connA.vehicle.getPosition(vid)
                    speed = connA.vehicle.getSpeed(vid)
                    angle = connA.vehicle.getAngle(vid)
                    road_id = connA.vehicle.getRoadID(vid)
                    veh_type = connA.vehicle.getTypeID(vid)

                    # Get lane info safely
                    try:
                        lane_id = connA.vehicle.getLaneID(vid)
                        lane_index = int(lane_id.split('_')[-1])
                    except:
                        lane_index = 0

                    # Check if GPS update is due
                    gps_due = (step - last_gps_update.get(vid, -gps_update_interval) >= gps_update_interval)

                    # Add vehicle to SimB if new
                    if vid not in seen_vehicles:
                        seen_vehicles.add(vid)
                        just_added_steps[vid] = step
                        
                        try:
                            route_edges = connA.vehicle.getRoute(vid)
                            route_id = f"{vid}_route"
                            
                            # Add route if doesn't exist
                            if route_id not in connB.route.getIDList():
                                connB.route.add(route_id, route_edges)
                            
                            # Add vehicle to SimB
                            connB.vehicle.add(vid, routeID=route_id, typeID=veh_type)
                            
                            # Set color (green for buses)
                            if veh_type == 'bus':
                                connB.vehicle.setColor(vid, (0, 255, 0, 255))
                            else:
                                connB.vehicle.setColor(vid, connA.vehicle.getColor(vid))
                            
                            # Initialize with current position immediately
                            connB.vehicle.moveToXY(vid, edgeID=road_id, laneIndex=lane_index, 
                                                 x=pos[0], y=pos[1], keepRoute=1)
                            connB.vehicle.setSpeed(vid, speed)
                            
                            print(f"[SPAWN] Added {vid} at ({pos[0]:.1f}, {pos[1]:.1f})")
                            
                        except Exception as e:
                            print(f"[ERROR] Could not add vehicle {vid}: {e}")
                            continue

                    # Short settling period - 2 steps
                    if step - just_added_steps.get(vid, 0) < 2:
                        continue

                    # Ensure vehicle exists in SimB
                    if vid not in connB.vehicle.getIDList():
                        continue

                    # Get vehicle state
                    current_speed, is_stopped = get_vehicle_state(connA, vid, stop_counter)
                    
                    # GPS update processing - ENHANCED WITH HYBRID APPROACH
                    if gps_due:
                        gps_x, gps_y, (true_x, true_y) = get_noisy_gps_position(connA, vid)
                        last_gps_update[vid] = step

                        if gps_x is not None and gps_y is not None:
                            try:
                                # Get current SimB position for logging
                                current_simb_pos = connB.vehicle.getPosition(vid)
                                lag_distance = euclidean((gps_x, gps_y), current_simb_pos)
                                
                                # APPLY HYBRID GPS MIRRORING (MAIN ENHANCEMENT)
                                success, intervention_type = hybrid_gps_mirroring(
                                    connA, connB, vid, (gps_x, gps_y), current_speed, is_stopped, step
                                )
                                
                                # Update position after hybrid processing for accurate logging
                                updated_simb_pos = connB.vehicle.getPosition(vid)
                                
                                # Enhanced logging with intervention type
                                gps_logger.writerow([step, vid, true_x, true_y, gps_x, gps_y, 
                                                   updated_simb_pos[0], updated_simb_pos[1], 
                                                   lag_distance, intervention_type])
                                
                                if not success:
                                    print(f"[FALLBACK] Hybrid approach failed for {vid}, using fallback")
                                    # Fallback to minimal positioning
                                    try:
                                        connB.vehicle.moveToXY(vid, edgeID=road_id, laneIndex=lane_index, 
                                                             x=gps_x, y=gps_y, keepRoute=1)
                                    except Exception as fallback_e:
                                        print(f"[ERROR] Fallback positioning failed for {vid}: {fallback_e}")
                                
                                # Periodic status logging for high lag
                                if lag_distance > 10 and step % 100 == 0:
                                    print(f"[HIGH_LAG] {vid}: lag={lag_distance:.1f}m, intervention={intervention_type}")
                                    
                            except Exception as e:
                                print(f"[ERROR] GPS processing failed for {vid}: {e}")
                                # Fallback to original simple positioning
                                try:
                                    connB.vehicle.moveToXY(vid, edgeID=road_id, laneIndex=lane_index, 
                                                         x=gps_x, y=gps_y, keepRoute=1)
                                except:
                                    pass
                    
                    else:
                        # Between GPS updates - maintain smooth operation
                        try:
                            # Check if vehicle is near traffic light for between-update handling
                            is_near_light, dist_to_light, light_state = detect_traffic_light_proximity(connB, vid)
                            
                            if is_vehicle_stopped_at_light(connB, vid):
                                # Vehicle stopped at light - let SUMO handle everything
                                pass
                            elif is_near_light and dist_to_light < HybridConfig.CRITICAL_LIGHT_DISTANCE:
                                # Close to light - let SUMO handle speed
                                pass
                            else:
                                # Safe area - continue GPS-based speed matching
                                if is_stopped:
                                    # Continue gradual stop
                                    current_simb_speed = connB.vehicle.getSpeed(vid)
                                    if current_simb_speed > 0.1:
                                        connB.vehicle.setSpeed(vid, current_simb_speed * 0.8)
                                    else:
                                        connB.vehicle.setSpeed(vid, 0)
                                else:
                                    # Maintain speed matching
                                    last_speed = vehicle_last_speed.get(vid, current_speed)
                                    smooth_speed = 0.8 * current_speed + 0.2 * last_speed
                                    connB.vehicle.setSpeed(vid, smooth_speed)
                                    vehicle_last_speed[vid] = smooth_speed
                        except Exception as e:
                            pass

                except Exception as e:
                    print(f"[ERROR] Processing vehicle {vid}: {e}")
                    continue

            # Handle pedestrians (simplified)
            for pid in connA.person.getIDList():
                try:
                    pos = connA.person.getPosition(pid)
                    edge = connA.person.getRoadID(pid)
                    if not edge:
                        continue
                        
                    if pid not in seen_vehicles:
                        seen_vehicles.add(pid)
                        connB.person.add(pid, edge, pos[0], 0)
                    
                    connB.person.moveToXY(pid, edge, x=pos[0], y=pos[1])
                except:
                    pass

            # Handle traffic lights - ENSURE SYNCHRONIZATION
            for tl_id in connA.trafficlight.getIDList():
                try:
                    state = connA.trafficlight.getRedYellowGreenState(tl_id)
                    phase = connA.trafficlight.getPhase(tl_id)
                    
                    # Synchronize both state and phase
                    connB.trafficlight.setRedYellowGreenState(tl_id, state)
                    try:
                        connB.trafficlight.setPhase(tl_id, phase)
                    except:
                        pass  # Phase setting might fail if programs differ
                        
                except Exception as e:
                    if step % 1000 == 0:  # Log traffic light errors less frequently
                        print(f"[TL_ERROR] Traffic light sync failed for {tl_id}: {e}")

            step += 1
            time.sleep(step_length)

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        print("Cleaning up simulation...")
        try:
            connA.close()
            connB.close()
            gps_log_file.close()
            close_camera_log()
        except:
            pass
        try:
            sumo_processA.terminate()
            sumo_processB.terminate()
        except:
            pass
        print("Cleanup complete.")

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Enhanced Digital Twin Urban Traffic Mirror with Traffic Light Compliance")
    parser.add_argument("--configA", required=True, help="SUMO config file for Simulation A")
    parser.add_argument("--configB", required=True, help="SUMO config file for Simulation B") 
    parser.add_argument("--portA", type=int, default=8813, help="Port for Simulation A")
    parser.add_argument("--portB", type=int, default=8814, help="Port for Simulation B")
    parser.add_argument("--steps", type=int, default=10000, help="Maximum simulation steps")
    parser.add_argument("--interval", type=float, default=0.1, help="Step length in seconds")
    
    # Configuration overrides
    parser.add_argument("--gps-error-threshold", type=float, default=HybridConfig.MIN_GPS_ERROR_FOR_CORRECTION,
                       help="Minimum GPS error before correction (meters)")
    parser.add_argument("--traffic-light-proximity", type=float, default=HybridConfig.TRAFFIC_LIGHT_PROXIMITY,
                       help="Distance to consider near traffic light (meters)")
    parser.add_argument("--near-light-correction", type=float, default=HybridConfig.NEAR_LIGHT_CORRECTION_FACTOR,
                       help="Correction factor when near traffic lights (0.0-1.0)")
    
    args = parser.parse_args()
    
    # Apply configuration overrides
    HybridConfig.MIN_GPS_ERROR_FOR_CORRECTION = args.gps_error_threshold
    HybridConfig.TRAFFIC_LIGHT_PROXIMITY = args.traffic_light_proximity
    HybridConfig.NEAR_LIGHT_CORRECTION_FACTOR = args.near_light_correction
    
    print(f"Starting Enhanced Digital Twin Mirror:")
    print(f"  SimA Config: {args.configA}")
    print(f"  SimB Config: {args.configB}")
    print(f"  GPS Update Interval: 5 steps")
    print(f"  Step Length: {args.interval}s")
    print(f"  GPS Error Threshold: {HybridConfig.MIN_GPS_ERROR_FOR_CORRECTION}m")
    print(f"  Traffic Light Proximity: {HybridConfig.TRAFFIC_LIGHT_PROXIMITY}m")
    print(f"  Near-Light Correction Factor: {HybridConfig.NEAR_LIGHT_CORRECTION_FACTOR}")
    print("=" * 60)
    
    mirror_simulation(args.configA, args.configB, args.portA, args.portB, args.steps, args.interval)

if __name__ == "__main__":
    main()