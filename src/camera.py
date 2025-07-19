# camera.py
import traci
import math
import random
import csv
import logging #+

#+ Get a logger instance for this module
logger = logging.getLogger(__name__)

# Define a virtual CCTV camera
CAMERAS = {
    "cctv_main": {
        "x": 1364.05,
        "y": 887.61,
        "range": 200  # meters
    },
    "cctv_top1": {
        "x": 2093.4,
        "y": 1409.50,
        "range": 200  # meters
    },
    "cctv_top2": {
        "x": 1897.34,
        "y": 1240.93,
        "range": 200  # meters
    }
}

# In-memory cache of last detection: vid → (step, cam_id, speed, lane, angle, noisy_dist)
CAMERA_CACHE = {}

# --- Data Logging (for analysis, separate from operational logging) ---
try:
    camera_log_file = open("camera_detections.csv", "w", newline='')
    log_writer = csv.writer(camera_log_file)
    log_writer.writerow(["step", "camera_id", "vehicle_id", "speed", "lane", "angle", "noisy_distance"])
except IOError:
    #+ Use logger for errors
    logger.error("Could not open camera_detections.csv for writing.")
    log_writer = None


def compute_noisy_distance(x1, y1, x2, y2, noise_factor=0.05):
    true_dist = math.hypot(x1 - x2, y1 - y2)
    noise = true_dist * noise_factor * random.uniform(-1, 1)
    return max(0, true_dist + noise)

def update_virtual_cameras(step, conn):
    for cam_id, cam in CAMERAS.items():
        for vid in conn.vehicle.getIDList():
            try:
                vx, vy = conn.vehicle.getPosition(vid)
                dist = math.hypot(vx - cam["x"], vy - cam["y"])

                if dist <= cam["range"]:
                    #-- This is a high-frequency event, so we use DEBUG level.
                    #-- It will appear in simulation.log but not on the console by default.
                    logger.debug(f"Vehicle {vid} detected by camera {cam_id} at distance {dist:.1f}m.")

                    lane = conn.vehicle.getLaneID(vid)
                    speed = conn.vehicle.getSpeed(vid)
                    angle = conn.vehicle.getAngle(vid)
                    
                    noisy_dist = compute_noisy_distance(cam["x"], cam["y"], vx, vy)
                    noisy_speed = speed * (1 + 0.02 * random.uniform(-1, 1))

                    # write to CSV data log
                    if log_writer:
                        log_writer.writerow([
                            step, cam_id, vid,
                            round(noisy_speed, 2), lane, round(angle, 1), round(noisy_dist, 2)
                        ])
                        
                    # update in-memory cache
                    CAMERA_CACHE[vid] = (
                        step, cam_id, round(noisy_speed, 2), lane,
                        round(angle, 1), round(noisy_dist, 2)
                    )
            except traci.TraCIException:
                # Vehicle might have departed, which is normal.
                continue

def close_camera_log():
    if camera_log_file:
        camera_log_file.close()
        logger.info("Camera detection data log closed.") #+