
import traci
import math
import random
import csv

# Define a virtual CCTV camera
CAMERAS = {
    "cctv_main": {
        "x": 1364.05,
        "y": 887.61,
        "range": 200  # meters
    }
}

# Logging setup
camera_log = open("camera_detections.csv", "w", newline='')
log_writer = csv.writer(camera_log)
log_writer.writerow(["step", "camera_id", "vehicle_id", "speed", "lane", "angle", "noisy_distance"])

def compute_noisy_distance(x1, y1, x2, y2, noise_factor=0.05):
    true_dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    noise = true_dist * noise_factor * random.uniform(-1, 1)
    return max(0, true_dist + noise)

def update_virtual_cameras(step,conn):
    for cam_id, cam in CAMERAS.items():
        for vid in conn.vehicle.getIDList():
            try:
                vx, vy = conn.vehicle.getPosition(vid)
                lane = conn.vehicle.getLaneID(vid)
                speed = conn.vehicle.getSpeed(vid)
                angle = conn.vehicle.getAngle(vid)

                # Euclidean distance
                dist = math.sqrt((vx - cam["x"])**2 + (vy - cam["y"])**2)
                if dist <= cam["range"]:
                    noisy_dist = compute_noisy_distance(cam["x"], cam["y"], vx, vy)
                    noisy_speed = speed + speed * 0.02 * random.uniform(-1, 1)
                    log_writer.writerow([
                        step, cam_id, vid,
                        round(noisy_speed, 2), lane, round(angle, 1), round(noisy_dist, 2)
                    ])
            except traci.TraCIException:
                continue

def close_camera_log():
    camera_log.close()
