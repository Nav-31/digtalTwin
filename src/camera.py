# camera.py
import traci
import math
import random
import csv
import logging

logger = logging.getLogger(__name__)

# --- CAMERAS and file setup are unchanged ---
CAMERAS = {
    "cctv_main1": { "x": 1364.05, "y": 887.61, "range": 200 },
    "cctv_main2": { "x":1479.90 , "y": 53.306543, "range": 200 },
     "cctv_truck": { "x": 1518.77, "y": 984.10, "range": 200 },
     "cctv_car": { "x": 1706.03, "y": 667.48, "range": 200 }
}


# The cache  stores: vid → (step, cam_id, noisy_speed, noisy_x, noisy_y)
CAMERA_CACHE = {}

try:
    camera_log_file = open("camera_detections.csv", "w", newline='')
    log_writer = csv.writer(camera_log_file)
    # --- MODIFIED CSV HEADER ---
    log_writer.writerow(["step", "camera_id", "vehicle_id", "noisy_speed", "noisy_x", "noisy_y"])
except IOError:
    logger.error("Could not open camera_detections.csv for writing.")
    log_writer = None



def update_virtual_cameras(step, conn):
    for cam_id, cam in CAMERAS.items():
        for vid in conn.vehicle.getIDList():
            try:
                vx, vy = conn.vehicle.getPosition(vid)
                dist = math.hypot(vx - cam["x"], vy - cam["y"])

                if dist <= cam["range"]:
                    logger.debug(f"Vehicle {vid} detected by camera {cam_id} at distance {dist:.1f}m.")
                    
                    speed = conn.vehicle.getSpeed(vid)
                    
                    # --- MODIFIED SENSOR LOGIC ---
                    # Simulate position error by adding Gaussian noise to the true coordinates.
                    # This is a more realistic model of camera position detection.
                    position_noise_std_dev = 2.0  # meters
                    noisy_x = vx + random.gauss(0, position_noise_std_dev)
                    noisy_y = vy + random.gauss(0, position_noise_std_dev)
                    
                    # Add noise to speed as before
                    noisy_speed = speed * (1 + 0.02 * random.uniform(-1, 1))

                    # write to CSV data log
                    if log_writer:
                        log_writer.writerow([
                            step, cam_id, vid, round(noisy_speed, 2), 
                            round(noisy_x, 2), round(noisy_y, 2)
                        ])
                        
                    # update in-memory cache with new format
                    CAMERA_CACHE[vid] = (
                        step, cam_id, round(noisy_speed, 2), 
                        noisy_x, noisy_y
                    )
            except traci.TraCIException:
                continue

def close_camera_log():
    if 'camera_log_file' in locals() and camera_log_file:
        camera_log_file.close()
        logger.info("Camera detection data log closed.")