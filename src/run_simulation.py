# run_simulation.py
# Enhanced Digital Twin Urban Traffic Mirror with Realistic Vehicle Behavior
# 
# DESIGN PRINCIPLES:
# 1. GPS vehicles (buses) use balanced GPS corrections to prevent lane jumping while maintaining responsiveness
# 2. Non-GPS vehicles (cars, trucks) drive naturally via SUMO with minimal camera intervention
# 3. Camera detection is used for spawning ALL vehicles when they enter camera range
# 4. Camera updates are only applied to non-GPS vehicles at junctions/intersections to avoid glitching
# 5. No random teleportation - maintains realistic vehicle movement at all times
# 6. Balanced corrections prevent vehicles from glitching while maintaining digital twin accuracy
# 7. Fast-moving vehicles (>15 m/s) skip GPS corrections to maintain natural highway driving
# 8. Robust error handling prevents simulation freezing
# 9. FIXED: Maintains consistent speed control (no aggressive releases) for proper Sim A/B synchronization
# 10. FIXED: Uses gentle speed adjustments inspired by old file approach for better digital twin sync
#
import os
import sys
import time
import subprocess
import traci
import random
import csv
import math
import logging

from logging_config import setup_logging
from camera import update_virtual_cameras, close_camera_log, CAMERA_CACHE, CAMERAS

logger = logging.getLogger(__name__)

# Global tracking for collision avoidance cooldowns to prevent emergency braking loops
collision_avoidance_cooldowns = {}



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
            
            # FIXED: Use old file's approach - more conservative minimum speed
            # Ensure minimum speed to prevent lag after spawning but maintain sync
            if smooth > 0 and smooth < 1.0:
                smooth = max(smooth, 1.0)  # Reduced from 1.5 to 1.0 for better sync
            
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
    MIN_CORRECTION_FACTOR = 0.1       # Back to more reasonable values
    MAX_CORRECTION_FACTOR = 0.5       # Increased from 0.2 but still conservative
    NEAR_LIGHT_CORRECTION_FACTOR = 0.3  # Increased from 0.1
    SPEED_THRESHOLD_STOPPED = 0.5
    SPEED_SMOOTH_FACTOR = 0.8           # Increased from 0.7 for better responsiveness
    GRADUAL_STOP_FACTOR = 0.8           # Increased from 0.7 for smoother stopping
    
    # Camera-specific parameters for non-GPS vehicles
    CAMERA_MIN_CORRECTION_DISTANCE = 2.0    # Minimum distance before applying camera correction (reduced from 5.0 to allow closer spawning)
    CAMERA_MAX_CORRECTION_DISTANCE = 100.0   # Increased maximum distance for camera correction to handle highway speeds
    CAMERA_CORRECTION_FACTOR = 0.35         # Increased responsiveness for better lag reduction
    CAMERA_SPEED_BLEND_FACTOR = 0.5         # Increased for more responsive speed updates
    
    # Collision avoidance parameters - FIXED: Made consistent for all vehicle types
    SAFE_FOLLOWING_DISTANCE = 12.0         # Increased minimum safe distance between vehicles
    COLLISION_WARNING_DISTANCE = 20.0      # Increased distance to start collision avoidance
    EMERGENCY_BRAKE_DISTANCE = 3.0         # Reduced critical distance for emergency braking (was too aggressive)
    LANE_CHANGE_SAFETY_DISTANCE = 25.0     # Increased safe distance for lane changes
    SPEED_REDUCTION_FACTOR = 0.7            # Less aggressive speed reduction factor

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
            det_step, cam_id, speed, noisy_x, noisy_y = det
            
            if det_step >= state['last_mirror_step']:
                logger.info(f"Using fresh camera detection for {vid} from {cam_id} (detected at step {det_step}).")
                
                # Check if this is a GPS vehicle - if so, don't interfere
                try:
                    vtype = connA.vehicle.getTypeID(vid)
                    if should_have_gps(vtype):
                        logger.debug(f"GPS vehicle {vid} detected by camera - skipping camera update (GPS takes priority).")
                        return False, 'camera_skipped_gps_vehicle'
                    
                    # Apply minimal collision avoidance check for non-GPS vehicles
                    collision_risk, risk_info = check_collision_risk(connB, vid)
                    if collision_risk == "critical":
                        logger.warning(f"COLLISION RISK: Non-GPS vehicle {vid} has critical collision risk - skipping camera update")
                        apply_collision_avoidance(connB, vid, collision_risk, risk_info)
                        return True, 'camera_collision_avoidance_critical'
                        
                except traci.TraCIException as e:
                    logger.debug(f"Could not check vehicle type/location for {vid}: {e}")
                    return False, 'camera_error_vehicle_check'
                
                # Check if camera position update is safe (simplified like old file)
                is_safe_update = True  # Default to allowing camera updates for responsiveness
                try:
                    # Only check for very close vehicles (< 2m) - much simpler safety check
                    for other_vid in connB.vehicle.getIDList():
                        if other_vid != vid:
                            try:
                                other_pos = connB.vehicle.getPosition(other_vid)
                                distance = euclidean((noisy_x, noisy_y), other_pos)
                                if distance < 2.0:  # Very conservative threshold
                                    is_safe_update = False
                                    break
                            except traci.TraCIException:
                                continue
                except:
                    is_safe_update = True  # If we can't check, allow the move for responsiveness
                
                if not is_safe_update:
                    logger.debug(f"Camera correction for non-GPS vehicle {vid} skipped due to very close vehicle")
                    return False, 'camera_skipped_close_vehicle'
                
                # Apply MINIMAL correction to avoid glitching
                # Get current position in mirror simulation
                try:
                    current_pos = connB.vehicle.getPosition(vid)
                    
                    # Calculate distance to camera detection
                    camera_distance = euclidean(current_pos, (noisy_x, noisy_y))
                    
                    # ENHANCED DISTANCE VALIDATION: Adjust thresholds based on vehicle type and speed
                    # Get vehicle type for intelligent distance validation
                    try:
                        vtype = connA.vehicle.getTypeID(vid)
                        current_speed = connB.vehicle.getSpeed(vid)
                        
                        # Dynamic distance thresholds based on vehicle type and speed
                        if vtype == 'taxi':
                            # Taxis need more flexible distance validation for highway speeds
                            min_distance = HybridConfig.CAMERA_MIN_CORRECTION_DISTANCE
                            max_distance = min(HybridConfig.CAMERA_MAX_CORRECTION_DISTANCE, max(50.0, current_speed * 8.0))  # Scale with speed
                        else:
                            # Standard validation for other vehicles  
                            min_distance = HybridConfig.CAMERA_MIN_CORRECTION_DISTANCE
                            max_distance = HybridConfig.CAMERA_MAX_CORRECTION_DISTANCE
                            
                    except:
                        # Fallback to standard validation
                        min_distance = HybridConfig.CAMERA_MIN_CORRECTION_DISTANCE
                        max_distance = HybridConfig.CAMERA_MAX_CORRECTION_DISTANCE
                    
                    # Only apply correction if the difference is significant but not too large
                    # This prevents small jitters and large teleportation glitches
                    if min_distance < camera_distance < max_distance:
                        # Apply minimal correction factor (similar to GPS near traffic lights)
                        correction_factor = HybridConfig.CAMERA_CORRECTION_FACTOR
                        corrected_x = current_pos[0] + correction_factor * (noisy_x - current_pos[0])
                        corrected_y = current_pos[1] + correction_factor * (noisy_y - current_pos[1])
                        
                        # DIRECTION VALIDATION: Prevent backward movement corrections
                        # Check if the correction is in a reasonable direction relative to current route
                        try:
                            route_edges = connB.vehicle.getRoute(vid)
                            current_edge = connB.vehicle.getRoadID(vid)
                            if route_edges and current_edge in route_edges:
                                # Simple direction check: ensure we're not moving dramatically backward
                                dx = corrected_x - current_pos[0]
                                dy = corrected_y - current_pos[1]
                                movement_distance = math.sqrt(dx*dx + dy*dy)
                                
                                # If movement is very small and speed is near zero, skip to prevent drift
                                current_speed = connB.vehicle.getSpeed(vid)
                                if movement_distance < 1.0 and current_speed < 0.5:
                                    logger.debug(f"Camera correction for {vid} skipped - vehicle nearly stopped with minimal movement to prevent backward drift.")
                                    return False, 'camera_prevented_backward_drift'
                        except traci.TraCIException:
                            pass  # If route check fails, continue with correction
                        
                        # Final safety check before moving - simplified for non-GPS vehicles
                        final_safety_check = True
                        try:
                            # Quick check for very close vehicles only
                            for other_vid in connB.vehicle.getIDList():
                                if other_vid != vid:
                                    try:
                                        other_pos = connB.vehicle.getPosition(other_vid)
                                        distance = euclidean((corrected_x, corrected_y), other_pos)
                                        if distance < 2.0:  # Very close threshold
                                            final_safety_check = False
                                            break
                                    except traci.TraCIException:
                                        continue
                        except:
                            final_safety_check = True
                            
                        if final_safety_check:
                            # Use conservative moveToXY with route adherence but not too restrictive
                            try:
                                connB.vehicle.moveToXY(
                                    vid, edgeID=state['road_id'], laneIndex=state['lane_index'],
                                    x=corrected_x, y=corrected_y, keepRoute=1  # Back to keepRoute=1 to prevent glitches
                                )
                                
                                # Apply speed smoothly to avoid jerky movements
                                current_speed = connB.vehicle.getSpeed(vid)
                                blend_factor = HybridConfig.CAMERA_SPEED_BLEND_FACTOR
                                smooth_speed = (1 - blend_factor) * current_speed + blend_factor * speed
                                
                                # ENHANCED ZERO SPEED LOCKUP PREVENTION: Critical for taxi vehicles
                                if speed > 0.5 and smooth_speed < 0.3:
                                    smooth_speed = max(smooth_speed, 0.5)  # Minimum movement speed to prevent getting stuck
                                    logger.debug(f"Camera correction for {vid}: Applied minimum speed {smooth_speed:.1f} to prevent zero-speed lockup.")
                                elif smooth_speed < 0.1:  # Critical lockup prevention
                                    # Get vehicle type to apply appropriate recovery speed
                                    try:
                                        vtype = connA.vehicle.getTypeID(vid)
                                        if vtype == 'taxi':
                                            recovery_speed = 2.0  # Higher recovery for taxis
                                        else:
                                            recovery_speed = 1.0  # Standard recovery
                                        smooth_speed = max(smooth_speed, recovery_speed)
                                        logger.warning(f"CRITICAL RECOVERY: Applied emergency speed {smooth_speed:.1f} to prevent {vid} ({vtype}) complete lockup")
                                    except:
                                        smooth_speed = max(smooth_speed, 1.0)  # Fallback recovery
                                
                                # Apply collision-aware speed if needed
                                if collision_risk == "moderate" or collision_risk == "high":
                                    smooth_speed = smooth_speed * HybridConfig.SPEED_REDUCTION_FACTOR
                                    apply_collision_avoidance(connB, vid, collision_risk, risk_info)
                                
                                connB.vehicle.setSpeed(vid, smooth_speed)
                                
                                # FIXED: Remove speed control release to maintain synchronization
                                # Keep consistent speed control like the old file for better digital twin sync
                                # No more aggressive speed releases that break synchronization
                                
                                logger.info(f"CAMERA_UPDATE: {vid} safely corrected to ({corrected_x:.1f}, {corrected_y:.1f}) with smooth speed {smooth_speed:.1f}.")
                                return True, 'camera_minimal_update'
                            except traci.TraCIException as e:
                                logger.warning(f"Camera correction failed for {vid}: {e}")
                                return False, 'camera_move_failed'
                        else:
                            logger.debug(f"Camera correction for {vid} cancelled by final safety check (vehicle within 2m)")
                            return False, 'camera_cancelled_close_vehicle'
                    else:
                        try:
                            vtype = connA.vehicle.getTypeID(vid)
                            current_speed = connB.vehicle.getSpeed(vid)
                            logger.debug(f"Camera detection for {vid} ({vtype}) distance {camera_distance:.1f}m outside range {min_distance:.1f}-{max_distance:.1f}m (speed: {current_speed:.1f} m/s) - skipping to avoid glitch.")
                        except:
                            logger.debug(f"Camera detection for {vid} distance {camera_distance:.1f}m outside validation range - skipping to avoid glitch.")
                        return False, 'camera_distance_rejected'
                        
                except traci.TraCIException as e:
                    logger.warning(f"Could not get current position for {vid} during camera update: {e}")
                    return False, 'camera_error'
                
        logger.debug(f"No fresh camera detection for {vid} at step {step}.")
        return False, 'camera_no_update'

class LoopMirroringStrategy(BaseMirroringStrategy):
    def mirror(self, connA, connB, vid, step, state):
        return False, 'loop_not_implemented'

class SumoNaturalStrategy(BaseMirroringStrategy):
    def mirror(self, connA, connB, vid, step, state):
        logger.debug(f"No sensor data for {vid}. Letting SUMO drive naturally.")
        # No intervention - let SUMO handle the vehicle naturally
        # This prevents glitching and maintains realistic movement
        return True, 'sumo_natural'

# Sequential fallback executor
def mirror_with_fallback(connA, connB, vid, step, state):
    # Try GPS first (for GPS-enabled vehicles)
    if state.get('gps_pos'):
        logger.debug(f"Attempting GPS mirror for {vid} at step {step}.")
        ok, mode = GPSMirroringStrategy().mirror(connA, connB, vid, step, state)
        if ok: return ok, mode
    
    # For non-GPS vehicles, try camera but apply strategic filtering
    logger.debug(f"Attempting camera mirror for {vid} at step {step}.")
    ok, mode = CameraMirroringStrategy().mirror(connA, connB, vid, step, state)
    if ok: return ok, mode
    
    # Try induction loops (future implementation)
    ok, mode = LoopMirroringStrategy().mirror(connA, connB, vid, step, state)
    if ok: return ok, mode
    
    # Final fallback: let SUMO drive naturally (no intervention)
    return SumoNaturalStrategy().mirror(connA, connB, vid, step, state)


# --- Utility Functions ---
def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def check_collision_risk(connB, vehicle_id):
    """
    Check if a vehicle is at risk of collision with nearby vehicles.
    Returns collision risk level and recommended action.
    """
    try:
        # Get vehicle's current state
        pos = connB.vehicle.getPosition(vehicle_id)
        speed = connB.vehicle.getSpeed(vehicle_id)
        lane_id = connB.vehicle.getLaneID(vehicle_id)
        
        if not lane_id:
            return "none", None
        
        # Check vehicles on the same lane
        vehicles_on_lane = []
        for other_vid in connB.vehicle.getIDList():
            if other_vid != vehicle_id:
                try:
                    other_lane = connB.vehicle.getLaneID(other_vid)
                    if other_lane == lane_id:
                        other_pos = connB.vehicle.getPosition(other_vid)
                        other_speed = connB.vehicle.getSpeed(other_vid)
                        distance = euclidean(pos, other_pos)
                        vehicles_on_lane.append({
                            'id': other_vid,
                            'pos': other_pos,
                            'speed': other_speed,
                            'distance': distance
                        })
                except traci.TraCIException:
                    continue
        
        # Find closest vehicle ahead
        closest_ahead = None
        min_ahead_distance = float('inf')
        
        for vehicle in vehicles_on_lane:
            # Simple ahead detection based on lane position
            # This is a simplified approach - in reality you'd need lane direction
            if vehicle['distance'] < min_ahead_distance and vehicle['distance'] > 1.0:
                # Check if vehicle is actually ahead by comparing positions
                # This is approximate but works for most cases
                dx = vehicle['pos'][0] - pos[0]
                dy = vehicle['pos'][1] - pos[1]
                # If vehicle is moving in roughly the same direction
                if abs(dx) > abs(dy):  # Horizontal movement
                    if (speed > 0 and dx > 0) or (speed < 0 and dx < 0):
                        closest_ahead = vehicle
                        min_ahead_distance = vehicle['distance']
                else:  # Vertical movement
                    if (speed > 0 and dy > 0) or (speed < 0 and dy < 0):
                        closest_ahead = vehicle
                        min_ahead_distance = vehicle['distance']
        
        # Determine collision risk level - FIXED: More consistent thresholds
        if closest_ahead:
            distance = closest_ahead['distance']
            relative_speed = speed - closest_ahead['speed']
            
            # Critical collision risk - only for very close vehicles
            if distance < HybridConfig.EMERGENCY_BRAKE_DISTANCE and relative_speed > 1.0:
                return "critical", {"action": "emergency_brake", "target_vehicle": closest_ahead['id'], "distance": distance}
            
            # High collision risk - need to slow down significantly
            elif distance < HybridConfig.SAFE_FOLLOWING_DISTANCE and relative_speed > 3.0:
                return "high", {"action": "reduce_speed", "target_vehicle": closest_ahead['id'], "distance": distance, "relative_speed": relative_speed}
            
            # Moderate collision risk - maintain safe following distance
            elif distance < HybridConfig.COLLISION_WARNING_DISTANCE and relative_speed > 0.5:
                return "moderate", {"action": "maintain_distance", "target_vehicle": closest_ahead['id'], "distance": distance}
        
        return "none", None
        
    except Exception as e:
        logger.warning(f"Error checking collision risk for {vehicle_id}: {e}")
        return "none", None

def apply_collision_avoidance(connB, vehicle_id, risk_level, risk_info):
    """
    Apply collision avoidance measures based on risk level.
    FIXED: Less aggressive and more consistent across all vehicle types.
    Includes cooldown mechanism to prevent emergency braking loops.
    """
    try:
        # Check cooldown to prevent repeated emergency interventions
        current_time = time.time()
        if vehicle_id in collision_avoidance_cooldowns:
            last_intervention = collision_avoidance_cooldowns[vehicle_id]
            # Cooldown period: 2 seconds for critical, 1 second for others
            cooldown_period = 2.0 if risk_level == "critical" else 1.0
            if current_time - last_intervention < cooldown_period:
                logger.debug(f"Collision avoidance cooldown active for {vehicle_id}, skipping intervention")
                return True
        
        current_speed = connB.vehicle.getSpeed(vehicle_id)
        
        if risk_level == "critical":
            # Emergency brake - but not as harsh
            new_speed = max(0, current_speed * 0.3)  # FIXED: Reduce to 30% instead of 10%
            connB.vehicle.setSpeed(vehicle_id, new_speed)
            logger.warning(f"COLLISION AVOIDANCE: Emergency brake for {vehicle_id} (distance: {risk_info['distance']:.1f}m)")
            collision_avoidance_cooldowns[vehicle_id] = current_time
            
        elif risk_level == "high":
            # Significant speed reduction - but gentler
            new_speed = max(0, current_speed * HybridConfig.SPEED_REDUCTION_FACTOR)
            connB.vehicle.setSpeed(vehicle_id, new_speed)
            logger.info(f"COLLISION AVOIDANCE: Speed reduction for {vehicle_id} (distance: {risk_info['distance']:.1f}m)")
            collision_avoidance_cooldowns[vehicle_id] = current_time
            
        elif risk_level == "moderate":
            # Gentle speed adjustment to maintain safe distance
            new_speed = max(0, current_speed * 0.85)  # FIXED: Less aggressive (was 0.8)
            connB.vehicle.setSpeed(vehicle_id, new_speed)
            logger.debug(f"COLLISION AVOIDANCE: Gentle slowdown for {vehicle_id} (distance: {risk_info['distance']:.1f}m)")
            # No cooldown for moderate interventions to allow frequent gentle adjustments
            
        return True
        
    except Exception as e:
        logger.warning(f"Failed to apply collision avoidance for {vehicle_id}: {e}")
        return False

def check_safe_position_update(connB, vehicle_id, target_x, target_y, gps_error=0):
    """
    Check if moving a vehicle to a target position would cause collisions.
    Returns True if safe, False if risky.
    Uses adaptive safety distances based on GPS error - larger errors get more lenient safety checks.
    """
    try:
        # Get current position to calculate GPS error if not provided
        if gps_error == 0:
            try:
                current_pos = connB.vehicle.getPosition(vehicle_id)
                gps_error = euclidean(current_pos, (target_x, target_y))
            except:
                gps_error = 0
        
        # Adaptive safety distance based on GPS error severity
        # For severe GPS errors (>30m), use very lenient safety to prevent coordinate corruption
        # For moderate errors (10-30m), use reduced safety distance
        # For small errors (<10m), use standard safety distance
        if gps_error > 30.0:
            min_distance = 3.0  # Emergency mode - only prevent immediate collisions
            logger.info(f"EMERGENCY GPS CORRECTION: Using minimal safety distance {min_distance}m for {vehicle_id} (GPS error: {gps_error:.1f}m)")
        elif gps_error > 15.0:
            min_distance = 6.0  # Reduced safety for moderate errors
            logger.debug(f"REDUCED SAFETY: Using {min_distance}m safety distance for {vehicle_id} (GPS error: {gps_error:.1f}m)")
        else:
            min_distance = HybridConfig.SAFE_FOLLOWING_DISTANCE  # Standard safety for small errors
        
        for other_vid in connB.vehicle.getIDList():
            if other_vid != vehicle_id:
                try:
                    other_pos = connB.vehicle.getPosition(other_vid)
                    distance_to_target = euclidean((target_x, target_y), other_pos)
                    
                    # If another vehicle is too close to our target position, it's not safe
                    if distance_to_target < min_distance:
                        if gps_error > 30.0:
                            # For severe GPS errors, log but allow anyway to prevent coordinate corruption
                            logger.warning(f"EMERGENCY OVERRIDE: Allowing GPS correction for {vehicle_id} despite {other_vid} at {distance_to_target:.1f}m (GPS error: {gps_error:.1f}m)")
                            return True
                        else:
                            logger.warning(f"Position update for {vehicle_id} blocked by {other_vid} at distance {distance_to_target:.1f}m")
                            return False
                        
                except traci.TraCIException:
                    continue
        
        return True
        
    except Exception as e:
        logger.warning(f"Error checking safe position for {vehicle_id}: {e}")
        # If we can't check safety, allow the move (fail-safe for non-GPS vehicles)
        return True

def should_have_gps(vtype):
    """
    Determines if a vehicle should have a functioning GPS based on its type.
    
    Args:
        vtype (str): The vehicle type ID (e.g., 'bus', 'taxi', 'car').
        
    Returns:
        bool: True if the vehicle should have GPS, False otherwise.
    """
    # Match the GPS configuration from dub.rou.xml where both bus and taxi have GPS enabled
    gps_enabled_types = {'bus', 'taxi'}
    result = vtype in gps_enabled_types
    if result:
        logger.debug(f"Vehicle type '{vtype}' confirmed as GPS-enabled")
    return result

def get_noisy_gps_position(conn, vehicle_id, std_dev=2.5):
    """
    Generates a noisy GPS position, but only for specific vehicle types.
    Returns None for vehicle types that are not GPS-enabled (e.g., 'car').
    """
    try:
        # First, get the vehicle's type ID from the simulation.
        vtype = conn.vehicle.getTypeID(vehicle_id)
        
        # Check if this vehicle type should have GPS enabled.
        if should_have_gps(vtype):
            # This vehicle is GPS-enabled, so generate noisy coordinates.
            logger.debug(f"Vehicle {vehicle_id} (type: {vtype}) has GPS. Generating signal.")
            true_pos = conn.vehicle.getPosition(vehicle_id)
            noisy_x = true_pos[0] + random.gauss(0, std_dev)
            noisy_y = true_pos[1] + random.gauss(0, std_dev)
            return noisy_x, noisy_y, true_pos
        else:
            # This vehicle type (e.g., 'car') does not have GPS.
            logger.debug(f"Vehicle {vehicle_id} (type: {vtype}) has NO GPS. Returning None.")
            true_pos = conn.vehicle.getPosition(vehicle_id)
            return None, None, true_pos
            
    except traci.TraCIException as e:
        # This can happen if the vehicle has already left the simulation.
        logger.warning(f"Could not get type for {vehicle_id} while checking for GPS: {e}")
        return None, None, (None, None)
    except Exception as e:
        logger.error(f"An unexpected error occurred in get_noisy_gps_position for {vehicle_id}: {e}")
        return None, None, (None, None)

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
    """MAIN HYBRID FUNCTION: Combines GPS positioning with traffic rule compliance and collision avoidance."""
    try:
        current_mirror_pos = connB.vehicle.getPosition(vehicle_id)
        current_mirror_speed = connB.vehicle.getSpeed(vehicle_id)
        gps_error = euclidean(gps_pos, current_mirror_pos)
        is_near_light_mirror, dist_mirror, state_mirror = detect_traffic_light_proximity(connB, vehicle_id)
        
        # COLLISION AVOIDANCE CHECK - Priority over GPS corrections
        collision_risk, risk_info = check_collision_risk(connB, vehicle_id)
        if collision_risk != "none":
            logger.info(f"COLLISION RISK: {vehicle_id} has {collision_risk} collision risk - applying avoidance")
            apply_collision_avoidance(connB, vehicle_id, collision_risk, risk_info)
            
            # If critical collision risk, skip GPS corrections entirely
            if collision_risk == "critical":
                return True, f"collision_avoidance_{collision_risk}"
        
        intervention_needed = False
        position_intervention = "none"
        
        # EMERGENCY RECOVERY: Check for catastrophic GPS errors that indicate coordinate corruption
        if gps_error > 100.0:  # Catastrophic error threshold
            logger.error(f"CATASTROPHIC GPS ERROR: {vehicle_id} has {gps_error:.1f}m error - attempting emergency recovery")
            try:
                # Force immediate correction to prevent further corruption
                road_id = connA.vehicle.getRoadID(vehicle_id)
                lane_id = connA.vehicle.getLaneID(vehicle_id)
                lane_index = int(lane_id.split('_')[-1]) if lane_id and '_' in lane_id else 0
                
                # Use minimal correction to prevent overcorrection spiral
                correction_factor = 0.3  # Conservative for emergency recovery
                corrected_x = current_mirror_pos[0] + correction_factor * (gps_pos[0] - current_mirror_pos[0])
                corrected_y = current_mirror_pos[1] + correction_factor * (gps_pos[1] - current_mirror_pos[1])
                
                # Emergency override - use SAME strategy for ALL GPS vehicles
                connB.vehicle.moveToXY(vehicle_id, edgeID=road_id, laneIndex=lane_index, 
                                     x=corrected_x, y=corrected_y, keepRoute=1)
                
                logger.info(f"EMERGENCY RECOVERY: Applied emergency correction to {vehicle_id}")
                return True, "emergency_recovery"
                
            except Exception as e:
                logger.error(f"Emergency recovery failed for {vehicle_id}: {e}")
                return False, "emergency_recovery_failed"

        # Be more conservative about GPS corrections to prevent lane jumping
        if gps_error > HybridConfig.MIN_GPS_ERROR_FOR_CORRECTION:
            # Get vehicle type for type-specific handling
            try:
                vtype = connA.vehicle.getTypeID(vehicle_id)
            except:
                vtype = "unknown"
            
            # Check if GPS position update is safe (no collisions) - pass GPS error for adaptive safety
            is_safe_update = check_safe_position_update(connB, vehicle_id, gps_pos[0], gps_pos[1], gps_error)
            if not is_safe_update:
                logger.warning(f"GPS correction for {vehicle_id} skipped due to collision risk")
                position_intervention = "no_correction_collision_risk"
            else:
                # Standard handling for GPS vehicles (buses)
                if current_mirror_speed > 15.0:  # Threshold for all GPS vehicles
                    position_intervention = "no_correction_fast_moving"
                    logger.debug(f"GPS: {vehicle_id} moving very fast ({current_mirror_speed:.1f} m/s), skipping correction to prevent issues.")
                elif is_vehicle_stopped_at_light(connB, vehicle_id):
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
            lane_index = int(lane_id.split('_')[-1]) if lane_id and '_' in lane_id else 0
            
            correction_factor = calculate_gps_correction_factor(gps_error)
            if position_intervention == "minimal_correction":
                correction_factor = min(correction_factor, HybridConfig.NEAR_LIGHT_CORRECTION_FACTOR)
            
            # Apply conservative correction but not overly restrictive
            corrected_x = current_mirror_pos[0] + correction_factor * (gps_pos[0] - current_mirror_pos[0])
            corrected_y = current_mirror_pos[1] + correction_factor * (gps_pos[1] - current_mirror_pos[1])
            
            # Double-check safety before applying correction - use adaptive safety based on GPS error
            if check_safe_position_update(connB, vehicle_id, corrected_x, corrected_y, gps_error):
                # Use the SAME strategy for GPS vehicles (buses)
                try:
                    # GPS vehicles use keepRoute=1 for consistent behavior
                    connB.vehicle.moveToXY(vehicle_id, edgeID=road_id, laneIndex=lane_index, 
                                         x=corrected_x, y=corrected_y, keepRoute=1)  # Same as buses
                    logger.debug(f"GPS correction applied to {vehicle_id}: factor={correction_factor:.2f}, lane={lane_index}")
                except traci.TraCIException as e:
                    logger.warning(f"Failed to apply GPS correction to {vehicle_id}: {e}")
                    # If correction fails, don't return - continue with speed control
            else:
                logger.warning(f"GPS correction for {vehicle_id} cancelled due to safety check")
                position_intervention = "no_correction_safety_check"

        # Speed control - with collision avoidance priority
        if collision_risk == "critical" or collision_risk == "high":
            # Collision avoidance already handled speed - don't override
            logger.debug(f"SPEED: Collision avoidance controlling speed for {vehicle_id}")
        elif is_vehicle_stopped_at_light(connB, vehicle_id) or (is_near_light_mirror and dist_mirror < HybridConfig.CRITICAL_LIGHT_DISTANCE):
            logger.debug(f"SPEED: Letting SUMO control speed for {vehicle_id} near traffic light.")
        else:
            if is_stopped:
                new_speed = current_mirror_speed * HybridConfig.GRADUAL_STOP_FACTOR if current_mirror_speed > 0.1 else 0
                connB.vehicle.setSpeed(vehicle_id, new_speed)
            else:
                smooth_speed = HybridConfig.SPEED_SMOOTH_FACTOR * current_speed + (1 - HybridConfig.SPEED_SMOOTH_FACTOR) * current_mirror_speed
                # Apply additional collision-aware speed reduction if moderate risk
                if collision_risk == "moderate":
                    smooth_speed = smooth_speed * 0.9  # 10% reduction for moderate risk
                
                # FIXED: Use old file's approach - more conservative minimum speed to maintain sync
                # Ensure minimum speed for moving vehicles to prevent lag/freezing but not too aggressive
                if smooth_speed > 0 and smooth_speed < 1.0:
                    smooth_speed = max(smooth_speed, 1.0)  # Reduced from 1.5 to 1.0 for better sync
                
                connB.vehicle.setSpeed(vehicle_id, smooth_speed)
        
        return True, position_intervention
        
    except traci.TraCIException as e:
        logger.warning(f"Hybrid mirroring failed for {vehicle_id}: {e}")
        return False, "error"
    except Exception as e:
        logger.error(f"Unexpected error in hybrid_gps_mirroring for {vehicle_id}: {e}", exc_info=True)
        return False, "error"


# --- Main Mirroring Loop  ---
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
    last_mirroring = {}
    gps_update_interval = 10
    stop_counter = {}
    vehicle_last_speed = {}

    # Setup for data analysis log
    gps_log_file = open("gps_vs_true_log.csv", "w", newline='')
    gps_logger = csv.writer(gps_log_file)
    gps_logger.writerow(["step", "bus_id", "true_x", "true_y", "noisy_x", "noisy_y", "simb_x", "simb_y", "lag_distance", "intervention_type"])

    try:
        while step < max_steps:
            connA.simulationStep()
            update_virtual_cameras(step, connA)
            connB.simulationStep()

            # --- STEP 1: GATHER ALL SENSOR DETECTIONS ---
            detected_vehicles_this_step = {}
            
            # GPS Scan (prioritized for GPS-enabled vehicles)
            for vid in connA.vehicle.getIDList():
                gps_x, gps_y, true_pos = get_noisy_gps_position(connA, vid)
                if gps_x is not None:
                    detected_vehicles_this_step[vid] = {
                        'source': 'gps',
                        'gps_pos': (gps_x, gps_y),
                        'true_pos': true_pos
                    }
            
            # Camera Scan (for all vehicles for spawning, but strategic updates for non-GPS)
            for vid, det in CAMERA_CACHE.items():
                if vid not in detected_vehicles_this_step and det[0] == step:
                    # For spawning: detect any vehicle under camera
                    # For updates: only non-GPS vehicles at junctions
                    try:
                        vtype = connA.vehicle.getTypeID(vid)
                        detected_vehicles_this_step[vid] = {
                            'source': 'camera',
                            'true_pos': connA.vehicle.getPosition(vid),
                            'vtype': vtype
                        }
                        logger.debug(f"Camera detection for vehicle {vid} (type: {vtype}).")
                    except traci.TraCIException:
                        pass  # Vehicle might have left simulation

            # --- STEP 2: PROCESS DETECTED VEHICLES (SPAWN OR UPDATE) ---
            for vid, data in detected_vehicles_this_step.items():
                
                # --- A. SENSOR-TRIGGERED SPAWNING ---
                if vid not in seen_vehicles:
                    try:
                        logger.info(f"FIRST DETECTION: Spawning vehicle {vid} via {data['source']} sensor.")
                        # Get static info from Sim A for spawning
                        pos = connA.vehicle.getPosition(vid)
                        speed = connA.vehicle.getSpeed(vid)
                        road_id = connA.vehicle.getRoadID(vid)
                        route_edges = connA.vehicle.getRoute(vid)
                        route_id = f"{vid}_route"
                        veh_type = connA.vehicle.getTypeID(vid)

                        connB.route.add(route_id, route_edges)
                        connB.vehicle.add(vid, routeID=route_id, typeID=veh_type)
                        
                        # Choose spawn position - prioritize natural flow over excessive safety for trucks
                        spawn_x, spawn_y = pos[0], pos[1]
                        
                        # For GPS vehicles, prefer GPS position but check safety
                        if data['source'] == 'gps' and data.get('gps_pos'):
                            gps_x, gps_y = data['gps_pos']
                            # Check if GPS position is safe for spawning
                            if check_safe_position_update(connB, vid, gps_x, gps_y):
                                spawn_x, spawn_y = gps_x, gps_y
                                logger.info(f"GPS vehicle {vid} safely spawned at GPS position ({gps_x:.1f}, {gps_y:.1f})")
                            else:
                                logger.warning(f"GPS position unsafe for {vid}, using true position instead")
                                spawn_x, spawn_y = pos[0], pos[1]
                        
                        # For trucks, be less restrictive with safety checks to prevent lag
                        if veh_type == 'truck':
                            # Trucks get priority for natural positioning - minimal safety override
                            spawn_x, spawn_y = pos[0], pos[1]  # Use exact true position
                            logger.info(f"Truck {vid} spawning at true position for optimal flow")
                        else:
                            # For other vehicles, do basic safety check
                            if not check_safe_position_update(connB, vid, spawn_x, spawn_y):
                                # Try to find a safer spawn position by offsetting slightly
                                for offset in [(-3, 0), (3, 0), (0, -3), (0, 3)]:  # Reduced offsets
                                    test_x, test_y = spawn_x + offset[0], spawn_y + offset[1]
                                    if check_safe_position_update(connB, vid, test_x, test_y):
                                        spawn_x, spawn_y = test_x, test_y
                                        logger.info(f"Found safer spawn position for {vid} with offset {offset}")
                                        break
                                else:
                                    logger.warning(f"Could not find safer spawn position for {vid}, using original position")
                        
                        # Spawn the vehicle
                        connB.vehicle.moveToXY(vid, edgeID=road_id, laneIndex=0, x=spawn_x, y=spawn_y, keepRoute=1)
                        
                        # FIXED: Use old file's approach - natural speed spawning for better synchronization
                        # Set initial speed using natural speeds without aggressive boosts or SUMO releases
                        if speed > 20.0:  # Only cap extremely high speeds
                            safe_speed = 15.0  # Reasonable highway speed
                        elif speed < 0.5:  # Handle very slow or stopped vehicles
                            # Give minimal momentum but don't be aggressive to maintain sync
                            safe_speed = max(speed, 1.0)  # Reduced from aggressive values for better sync
                        else:
                            safe_speed = speed  # Use actual speed for natural movement
                        
                        # FIXED: Set speed and maintain control like old file - no aggressive releases
                        connB.vehicle.setSpeed(vid, safe_speed)
                        
                        # FIXED: Remove aggressive speed control releases and acceleration manipulation
                        # Keep consistent speed control for all vehicle types like the old file
                        # This maintains better synchronization between Sim A and Sim B
                        
                        logger.info(f"Spawned {veh_type} {vid} with natural speed {safe_speed:.1f} m/s (original: {speed:.1f} m/s)")
                        
                        # Set consistent colors for all vehicle types to match Sim A
                        if veh_type == 'bus':
                            connB.vehicle.setColor(vid, (0, 255, 0, 255))  # Green for buses
                        elif veh_type == 'truck':
                            connB.vehicle.setColor(vid, (0, 0, 255, 255))  # Blue for trucks
                        elif veh_type == 'car':
                            connB.vehicle.setColor(vid, (255, 0, 0, 255))  # Red for cars
                        else:
                            # Fallback to simulation A color for unknown types
                            connB.vehicle.setColor(vid, connA.vehicle.getColor(vid))

                        seen_vehicles.add(vid)
                        last_mirroring[vid] = step # Mark as just updated
                        
                        # For trucks, mark spawn time to avoid early interference
                        if veh_type == 'truck':
                            if not hasattr(mirror_simulation, 'truck_spawn_times'):
                                mirror_simulation.truck_spawn_times = {}
                            mirror_simulation.truck_spawn_times[vid] = step
                            logger.info(f"Marked truck {vid} spawn time for protection from early interference")
                    except traci.TraCIException as e:
                        logger.error(f"Failed to spawn detected vehicle {vid}: {e}")
                        continue
                
                # --- B. UPDATE EXISTING VEHICLES ---
                # GPS vehicles get regular updates, non-GPS vehicles get updates only when detected by cameras at strategic locations
                vtype = connA.vehicle.getTypeID(vid)
                is_gps_vehicle = should_have_gps(vtype)
                
                if is_gps_vehicle:
                    # ALL GPS vehicles: same regular interval updates
                    is_mirroring_due = (step - last_mirroring.get(vid, -gps_update_interval) >= gps_update_interval)
                else:
                    # Non-GPS vehicles: update when camera detects them (simplified like old file)
                    if data['source'] == 'camera':
                        # Simple frequency check - same as GPS vehicles for consistency
                        last_camera_update = last_mirroring.get(vid, -gps_update_interval)
                        time_since_last_update = step - last_camera_update
                        
                        # Update at same frequency as GPS vehicles but using camera data
                        is_mirroring_due = time_since_last_update >= gps_update_interval  # Same 10-step interval as GPS
                        if is_mirroring_due:
                            logger.debug(f"Non-GPS vehicle {vid} camera update due (last update: {time_since_last_update} steps ago).")
                        else:
                            logger.debug(f"Non-GPS vehicle {vid} detected by camera but too soon since last update ({time_since_last_update} steps).")
                    else:
                        is_mirroring_due = False
                
                if is_mirroring_due:
                    last_mirroring[vid] = step
                    try:
                        # Get fresh state from Sim A for the update
                        true_pos = connA.vehicle.getPosition(vid)
                        speed, is_stopped = get_vehicle_state(connA, vid, stop_counter)
                        road_id = connA.vehicle.getRoadID(vid)
                        lane_id = connA.vehicle.getLaneID(vid)
                        lane_index = int(lane_id.split('_')[-1]) if lane_id and '_' in lane_id else 0
                        
                        state = {
                            'speed': speed, 'is_stopped': is_stopped, 'true_pos': true_pos,
                            'road_id': road_id, 'lane_index': lane_index,
                            'last_mirror_step': last_mirroring.get(vid, 0),
                            'camera_cache': CAMERA_CACHE, 'gps_pos': data.get('gps_pos')
                        }

                        success, intervention_type = mirror_with_fallback(connA, connB, vid, step, state)
                        
                        # Log data for analysis
                        try:
                            if connB.vehicle.getIDList() and vid in connB.vehicle.getIDList():
                                updated_pos = connB.vehicle.getPosition(vid)
                                lag = euclidean(true_pos, updated_pos)
                                gps_logger.writerow([step, vid, true_pos[0], true_pos[1], state['gps_pos'][0] if state['gps_pos'] else None, state['gps_pos'][1] if state['gps_pos'] else None, updated_pos[0], updated_pos[1], lag, intervention_type])
                        except Exception as e:
                            logger.debug(f"Could not log data for {vid}: {e}")
                            # Don't let logging errors break the simulation
                    except traci.TraCIException as e:
                        logger.warning(f"Could not update vehicle {vid}: {e}")
                        # Vehicle might have left simulation - remove from tracking
                        seen_vehicles.discard(vid)
                    except Exception as e:
                        logger.error(f"Unexpected error updating vehicle {vid}: {e}")
                        # Don't let one vehicle error break entire simulation
                else:
                    # Vehicle exists but no update needed - let it continue naturally
                    logger.debug(f"Vehicle {vid} ({vtype}) continuing natural movement - no sensor update needed.")

            # --- STEP 3: HANDLE UN-DETECTED BUT EXISTING VEHICLES ---
            undetected_vids = seen_vehicles - detected_vehicles_this_step.keys()
            for vid in list(undetected_vids):  # Use list() to avoid modification during iteration
                try:
                    # Check if vehicle still exists in both simulations
                    exists_in_A = vid in connA.vehicle.getIDList()
                    exists_in_B = vid in connB.vehicle.getIDList()
                    
                    if not exists_in_A:
                        logger.info(f"Vehicle {vid} departed from Sim A - removing from tracking")
                        seen_vehicles.discard(vid)
                        continue
                    
                    if not exists_in_B:
                        logger.warning(f"Vehicle {vid} missing from Sim B but exists in Sim A - attempting respawn")
                        # Try to respawn the vehicle in Sim B
                        try:
                            pos = connA.vehicle.getPosition(vid)
                            speed = connA.vehicle.getSpeed(vid)
                            road_id = connA.vehicle.getRoadID(vid)
                            route_edges = connA.vehicle.getRoute(vid)
                            route_id = f"{vid}_route_respawn"
                            veh_type = connA.vehicle.getTypeID(vid)
                            
                            connB.route.add(route_id, route_edges)
                            connB.vehicle.add(vid, routeID=route_id, typeID=veh_type)
                            connB.vehicle.moveToXY(vid, edgeID=road_id, laneIndex=0, x=pos[0], y=pos[1], keepRoute=1)
                            
                            # FIXED: Use natural speed for respawned vehicles - old file approach
                            if speed > 20.0:
                                respawn_speed = 15.0
                            elif speed < 0.5:
                                # Use minimal boost for all vehicle types to maintain sync
                                respawn_speed = max(speed, 1.0)  # Reduced from aggressive values
                            else:
                                respawn_speed = speed
                                
                            connB.vehicle.setSpeed(vid, respawn_speed)
                            
                            # FIXED: Remove speed control release - maintain control like old file
                            # Keep consistent speed control for better synchronization
                            
                            # Set vehicle colors to match Sim A
                            if veh_type == 'bus':
                                connB.vehicle.setColor(vid, (0, 255, 0, 255))  # Green for buses
                            elif veh_type == 'truck':
                                connB.vehicle.setColor(vid, (0, 0, 255, 255))  # Blue for trucks
                            elif veh_type == 'car':
                                connB.vehicle.setColor(vid, (255, 0, 0, 255))  # Red for cars
                            
                            logger.info(f"Successfully respawned {veh_type} {vid} with natural speed {respawn_speed:.1f} m/s")
                        except Exception as respawn_e:
                            logger.error(f"Failed to respawn {vid} in Sim B: {respawn_e}")
                            seen_vehicles.discard(vid)
                        continue
                    
                    # Both vehicles exist - continue with normal handling
                    # Check if this is a GPS vehicle - if so, apply smoothing
                    vtype = connA.vehicle.getTypeID(vid)
                    if should_have_gps(vtype):
                        # GPS vehicles that are temporarily not detected - apply gentle smoothing
                        speed, is_stopped = get_vehicle_state(connA, vid, stop_counter)
                        state = {'speed': speed, 'is_stopped': is_stopped}
                        handle_between_update(connB, vid, state, vehicle_last_speed)
                    else:
                        # Non-GPS vehicles (cars, trucks) - let SUMO drive naturally without ANY intervention
                        # This prevents glitching and maintains realistic movement
                        # No speed control, no position control - pure SUMO behavior
                        logger.debug(f"Non-GPS vehicle {vid} ({vtype}) driving naturally via SUMO (no sensor intervention)")
                        
                        # FIXED: Apply old file approach - gentle smoothing instead of aggressive boosts
                        # However, ensure they're not stuck at very low speeds from spawning
                        try:
                            current_speed = connB.vehicle.getSpeed(vid)
                            if 0 < current_speed < 1.0:  # Reduced threshold for better sync
                                # Apply gentle speed adjustment for all vehicle types - no aggressive boosts
                                gentle_speed = max(current_speed, 1.0)  # Minimal boost, same for all types
                                connB.vehicle.setSpeed(vid, gentle_speed)
                                logger.debug(f"Gentle speed adjustment for {vtype} {vid}: {current_speed:.1f} -> {gentle_speed:.1f} m/s")
                        except traci.TraCIException:
                            pass  # If we can't check/set speed, let SUMO handle it
                        
                except traci.TraCIException:
                    logger.warning(f"Could not find vehicle {vid} in Sim A. It may have departed.")
                    seen_vehicles.discard(vid) # Remove from seen set if it's gone
                except Exception as e:
                    logger.debug(f"Error handling undetected vehicle {vid}: {e}")
                    seen_vehicles.discard(vid)

            # --- STEP 4: COLLISION MONITORING AND EMERGENCY INTERVENTION ---
            # Monitor only GPS vehicles for collision risks to avoid interfering with natural SUMO driving
            current_vehicles = set(connB.vehicle.getIDList())
            
            # Clean up cooldowns for vehicles that no longer exist
            for vid in list(collision_avoidance_cooldowns.keys()):
                if vid not in current_vehicles:
                    del collision_avoidance_cooldowns[vid]
            
            for vid in list(current_vehicles):
                if vid in seen_vehicles:
                    try:
                        # Only apply collision avoidance to GPS vehicles
                        # Non-GPS vehicles should drive naturally via SUMO
                        vtype = connA.vehicle.getTypeID(vid) if vid in connA.vehicle.getIDList() else None
                        
                        # Special protection for newly spawned trucks - give them time to accelerate
                        if vtype == 'truck' and hasattr(mirror_simulation, 'truck_spawn_times'):
                            spawn_time = mirror_simulation.truck_spawn_times.get(vid, 0)
                            if step - spawn_time < 50:  # Protect trucks for first 50 steps (5 seconds)
                                logger.debug(f"Protecting newly spawned truck {vid} from collision interference")
                                continue
                        
                        if vtype and should_have_gps(vtype):
                            collision_risk, risk_info = check_collision_risk(connB, vid)
                            if collision_risk == "critical" or collision_risk == "high":
                                # FIXED: Apply same logic to GPS vehicles as non-GPS vehicles
                                logger.info(f"COLLISION RISK: GPS vehicle {vid} has {collision_risk} collision risk - applying avoidance")
                                apply_collision_avoidance(connB, vid, collision_risk, risk_info)
                            elif collision_risk == "moderate":
                                # Apply gentle avoidance for moderate risk
                                apply_collision_avoidance(connB, vid, collision_risk, risk_info)
                        # Non-GPS vehicles are left to SUMO's natural collision avoidance
                        
                    except Exception as e:
                        logger.debug(f"Error in collision monitoring for {vid}: {e}")

            # Synchronize Traffic Lights
            for tl_id in connA.trafficlight.getIDList():
                try:
                    state = connA.trafficlight.getRedYellowGreenState(tl_id)
                    connB.trafficlight.setRedYellowGreenState(tl_id, state)
                except traci.TraCIException:
                    logger.warning(f"Could not sync traffic light {tl_id}. It may not exist in Sim B.")
            
            step += 1
            time.sleep(step_length)

    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user.")
    except Exception as e:
        logger.critical(f"A critical error occurred in the main loop: {e}", exc_info=True)
    finally:
        logger.info("Cleaning up and closing connections...")
        if 'connA' in locals(): connA.close()
        if 'connB' in locals(): connB.close()
        if 'gps_log_file' in locals() and not gps_log_file.closed: gps_log_file.close()
        close_camera_log()
        if 'sumo_processA' in locals(): sumo_processA.terminate()
        if 'sumo_processB' in locals(): sumo_processB.terminate()
        logger.info("Cleanup complete. Simulation finished.")


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

    log_level = logging.DEBUG if args.verbose else logging.INFO
    setup_logging(console_level=log_level)

    mirror_simulation(args.configA, args.configB, args.portA, args.portB, args.steps, args.interval)

if __name__ == "__main__":
    main()