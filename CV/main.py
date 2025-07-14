# hand_dodge_direction_filtered.py
#
# Requirements:
#   pip install opencv-python mediapipe numpy
#
# Usage:
#   python hand_dodge_direction_filtered.py

import cv2
import mediapipe as mp
import numpy as np
import time
import csv

# ─── Parameters ───────────────────────────────────────────────────────────────

FRAME_W, FRAME_H   = 640, 480

THREAT_DIST_PX     = 150    # how close the hand’s path must come to rod
DODGE_DIST_PX      = 100   # length of dodge-vector for direction calc

ALPHA              = 0.7   # velocity smoothing factor

# Maximum allowed red-arrow magnitude (px). Bigger than this is treated as noise.
MAX_RED_DODGE_MAG  = 10   

# rod at frame center
ROD_POS = np.array([335, 225])

# ─── Helper Functions ─────────────────────────────────────────────────────────

def palm_center(landmarks, w, h):
    pts = np.array([
        [landmarks[0].x * w,  landmarks[0].y * h],
        [landmarks[5].x * w,  landmarks[5].y * h],
        [landmarks[17].x * w, landmarks[17].y * h],
    ])
    return pts.mean(axis=0)

def threat_and_dodge(v, hand_pos):
    to_rod      = ROD_POS - hand_pos
    approaching = (v @ to_rod) > 0
    speed       = np.linalg.norm(v) + 1e-6
    dist_line   = abs(np.cross(v, to_rod) / speed)
    is_threat   = approaching and (dist_line < THREAT_DIST_PX)
    dodge       = np.zeros(2)
    if is_threat:
        # Try both perpendicular directions
        perp1 = np.array([v[1], -v[0]]) / speed  # clockwise
        perp2 = np.array([-v[1], v[0]]) / speed  # counterclockwise

        # Evaluate which one escapes farther from rod
        test1 = hand_pos + perp1 * DODGE_DIST_PX
        test2 = hand_pos + perp2 * DODGE_DIST_PX
        dist1 = np.linalg.norm(ROD_POS - test1)
        dist2 = np.linalg.norm(ROD_POS - test2)

        best_perp = perp1 if dist1 > dist2 else perp2
        dodge = best_perp * DODGE_DIST_PX

    print(dodge)
    return is_threat, dodge

def dodge_direction(vec):
    dx, dy = vec
    dirs   = []
    if dx > 0:      dirs.append("right")
    elif dx < 0:    dirs.append("left")
    if dy > 0:      dirs.append("down")
    elif dy < 0:    dirs.append("up")
    return "-".join(dirs) if dirs else "none"

# def find_impact_frame(logs, threat_dist):
#     """
#     logs: list of dicts with keys "frame" and "distance"
#     threat_dist: numeric threshold
#     returns: integer frame index of first impact
#     """
#     for entry in logs:
#         dist = entry["distance"]
#         if dist is not None and dist <= threat_dist:
#             return entry["frame"]
#     raise ValueError("No impact frame found: hand never crossed threshold")

# def find_detection_frame(logs):
#     """
#     logs: list of dicts with keys "frame", "threat" (bool), "dodge_mag" (float)
#     returns: integer frame index of first dodge decision
#     """
#     for entry in logs:
#         if entry["threat"] and entry["dodge_mag"] > 0:
#             return entry["frame"]
#     raise ValueError("No dodge decision found")

# ─── Main Loop ────────────────────────────────────────────────────────────────

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    mp_hands = mp.solutions.hands
    hands    = mp_hands.Hands(
                  static_image_mode=False,
                  max_num_hands=1,
                  min_detection_confidence=0.7,
                  min_tracking_confidence=0.7
               )

    last_c, last_t, v_prev = None, None, np.zeros(2, float)

    frame_idx = 0
    logs = []  # list of dicts: one entry per frame

    print("▶ Press 'q' to quit")
    while True:
        # frame_idx += 1

        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res      = hands.process(rgb)

        # default
        threat, dodge = False, np.zeros(2)

        if res.multi_hand_landmarks:
            lm     = res.multi_hand_landmarks[0].landmark
            center = palm_center(lm, w, h)
            cv2.circle(frame, tuple(center.astype(int)), 5, (0,255,0), -1)

            # 1) velocity estimation
            now = time.time()
            if last_c is not None:
                dt = now - last_t
                if dt > 0:
                    v_raw = (center - last_c) / dt
                else:
                    v_raw = v_prev.copy()
            else:
                v_raw = v_prev.copy()

            v = ALPHA * v_raw + (1 - ALPHA) * v_prev

            # 2) draw blue arrow = velocity direction
            end_v = (center + v * 0.05).astype(int)
            cv2.arrowedLine(frame,
                            tuple(center.astype(int)),
                            tuple(end_v),
                            (255,0,0), 2, tipLength=0.3)

            # 3) threat + red dodge
            threat, dodge = threat_and_dodge(v, center)
            mag = np.linalg.norm(dodge)
            if threat and mag <= MAX_RED_DODGE_MAG:
                end_d = (center + dodge).astype(int)
                cv2.arrowedLine(frame,
                                tuple(center.astype(int)),
                                tuple(end_d),
                                (0,0,255), 2, tipLength=0.3)
                print(dodge_direction(dodge))
            
            dist_to_rod = np.linalg.norm(ROD_POS - center) if center is not None else None
            # logs.append({
            # "frame":      frame_idx,
            # "distance":   dist_to_rod,
            # "threat":     bool(threat),
            # "dodge_mag":  float(np.linalg.norm(dodge)),
            # })

            # update history
            last_c, last_t, v_prev = center.copy(), now, v.copy()
        else:
            last_c, last_t, v_prev = None, None, np.zeros(2, float)

        # draw rod
        cv2.circle(frame, tuple(ROD_POS.astype(int)), 5, (255,255,0), -1)
        cv2.imshow("Hand Dodge Direction", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # with open("dodge_log.csv", "w", newline="") as f:
    #     writer = csv.DictWriter(f, fieldnames=["frame","distance","threat","dodge_mag"])
    #     writer.writeheader()
    #     writer.writerows(logs)

    # F_impact = find_impact_frame(logs, THREAT_DIST_PX)
    # F_detect = find_detection_frame(logs)
    # lead_frames = F_impact - F_detect
    # print(F_impact, F_detect, lead_frames)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
