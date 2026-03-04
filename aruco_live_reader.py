#!/usr/bin/env python3
import argparse
import time

import cv2
import numpy as np


def parse_id_color_map(text):
    out = {}
    if not text.strip():
        return out
    pairs = [p.strip() for p in text.split(",") if p.strip()]
    for p in pairs:
        if ":" not in p:
            continue
        a, b = p.split(":", 1)
        try:
            marker_id = int(a.strip())
        except ValueError:
            continue
        out[marker_id] = b.strip().lower()
    return out


def parse_id_list(text):
    out = []
    if not text.strip():
        return out
    parts = [p.strip() for p in text.split(",") if p.strip()]
    for p in parts:
        try:
            out.append(int(p))
        except ValueError:
            continue
    # Keep deterministic unique order.
    return sorted(set(out))


def get_aruco_detector(dictionary_name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco module not found. Install opencv-contrib-python.")

    dict_id = getattr(cv2.aruco, dictionary_name, None)
    if dict_id is None:
        raise RuntimeError(f"Unknown dictionary '{dictionary_name}'. Example: DICT_4X4_50")

    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)

        def detect(frame):
            corners, ids, _ = detector.detectMarkers(frame)
            return corners, ids

    else:

        def detect(frame):
            corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=params)
            return corners, ids

    return detect


def build_camera_matrix(width, height, fx, fy, cx, cy, hfov_deg):
    if fx > 0.0 and fy > 0.0:
        cam_fx = float(fx)
        cam_fy = float(fy)
        cam_cx = float(cx) if cx > 0.0 else (width / 2.0)
        cam_cy = float(cy) if cy > 0.0 else (height / 2.0)
    else:
        # Fallback estimate when no camera calibration is provided.
        hfov_rad = float(hfov_deg) * np.pi / 180.0
        cam_fx = (width / 2.0) / np.tan(hfov_rad / 2.0)
        cam_fy = cam_fx
        cam_cx = width / 2.0
        cam_cy = height / 2.0

    k = np.array(
        [
            [cam_fx, 0.0, cam_cx],
            [0.0, cam_fy, cam_cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    d = np.zeros((5, 1), dtype=np.float32)
    return k, d


def estimate_marker_distance_m(marker_corners, marker_size_m, camera_matrix, dist_coeffs):
    corners = marker_corners.astype(np.float32).reshape(1, 4, 2)

    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size_m, camera_matrix, dist_coeffs)
        if tvecs is None or len(tvecs) == 0:
            return None
        t = tvecs[0][0]
        return float(np.linalg.norm(t))

    half = marker_size_m / 2.0
    obj = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    img = marker_corners.reshape(4, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    t = tvec.reshape(3)
    return float(np.linalg.norm(t))


def main():
    parser = argparse.ArgumentParser(description="Live ArUco reader using laptop camera")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--dictionary", type=str, default="DICT_4X4_50", help="ArUco dictionary name")
    parser.add_argument("--expected-ids", type=str, default="", help="Comma-separated marker IDs to verify (example: 0,1,2)")
    parser.add_argument("--id-colors", type=str, default="", help="Optional mapping 'id:color,id:color' for overlay labels")
    parser.add_argument("--min-perimeter", type=float, default=80.0, help="Minimum marker perimeter in pixels")
    parser.add_argument("--marker-size-m", type=float, default=0.05, help="Marker side length in meters (default: 0.05)")
    parser.add_argument("--fx", type=float, default=0.0, help="Camera fx in pixels (optional)")
    parser.add_argument("--fy", type=float, default=0.0, help="Camera fy in pixels (optional)")
    parser.add_argument("--cx", type=float, default=0.0, help="Camera cx in pixels (optional)")
    parser.add_argument("--cy", type=float, default=0.0, help="Camera cy in pixels (optional)")
    parser.add_argument("--hfov-deg", type=float, default=70.0, help="Horizontal FOV degrees used if fx/fy are not provided")
    parser.add_argument("--stable-frames", type=int, default=8, help="Frames required for stable detection")
    parser.add_argument("--exit-on-pass", action="store_true", help="Exit once all expected IDs are stably detected")
    args = parser.parse_args()

    id_color = parse_id_color_map(args.id_colors)
    expected_ids = parse_id_list(args.expected_ids)
    if not expected_ids and id_color:
        expected_ids = sorted(id_color.keys())

    detect_fn = get_aruco_detector(args.dictionary)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera index {args.camera}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    stable_counts = {i: 0 for i in expected_ids}
    last_log_t = 0.0
    k, d = build_camera_matrix(
        width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width,
        height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height,
        fx=args.fx,
        fy=args.fy,
        cx=args.cx,
        cy=args.cy,
        hfov_deg=args.hfov_deg,
    )

    print("ArUco reader started")
    print(f"dictionary={args.dictionary} expected_ids={expected_ids}")
    if args.fx <= 0.0 or args.fy <= 0.0:
        print(f"distance mode: using estimated intrinsics from hfov={args.hfov_deg} deg (less accurate)")
    else:
        print("distance mode: using provided fx/fy/cx/cy intrinsics")
    print(f"marker_size_m={args.marker_size_m}")
    if not expected_ids:
        print("No expected IDs configured: running in monitor-only mode")
    print("Press 'q' to quit")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue

            corners, ids = detect_fn(frame)

            seen_ids = []
            kept = []
            marker_distances = {}
            if ids is not None:
                for marker_id, marker_corners in zip(ids.flatten().tolist(), corners):
                    peri = cv2.arcLength(marker_corners.astype("float32"), True)
                    if peri < args.min_perimeter:
                        continue
                    seen_ids.append(int(marker_id))
                    kept.append((int(marker_id), marker_corners))
                    dist_m = estimate_marker_distance_m(marker_corners, args.marker_size_m, k, d)
                    if dist_m is not None:
                        marker_distances[int(marker_id)] = dist_m

            seen_set = set(seen_ids)

            for marker_id in expected_ids:
                if marker_id in seen_set:
                    stable_counts[marker_id] += 1
                else:
                    stable_counts[marker_id] = 0

            for marker_id, marker_corners in kept:
                pts = marker_corners.reshape(-1, 2).astype(int)
                cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 255), thickness=2)
                c = pts.mean(axis=0).astype(int)
                color = id_color.get(marker_id, "unknown")
                if marker_id in stable_counts:
                    stable = stable_counts.get(marker_id, 0)
                    dist_m = marker_distances.get(marker_id)
                    if marker_id in id_color:
                        text = f"id={marker_id} {color} st={stable}"
                    else:
                        text = f"id={marker_id} st={stable}"
                    if dist_m is not None:
                        text += f" d={dist_m:.2f}m"
                else:
                    dist_m = marker_distances.get(marker_id)
                    if marker_id in id_color:
                        text = f"id={marker_id} {color}"
                    else:
                        text = f"id={marker_id}"
                    if dist_m is not None:
                        text += f" d={dist_m:.2f}m"
                cv2.putText(frame, text, (int(c[0]), int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

            all_pass = all(stable_counts.get(i, 0) >= args.stable_frames for i in expected_ids) if expected_ids else False
            status = "PASS" if all_pass else ("WAIT" if expected_ids else "MONITOR")
            cv2.putText(
                frame,
                f"{status} seen={sorted(seen_set)} stable={stable_counts if expected_ids else '{}'}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if all_pass else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            now = time.time()
            if now - last_log_t > 1.0:
                dist_log = {mid: round(marker_distances[mid], 3) for mid in sorted(marker_distances.keys())}
                print(f"seen={sorted(seen_set)} stable={stable_counts} dist_m={dist_log} status={status}")
                last_log_t = now

            cv2.imshow("ArUco Live Reader", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            if all_pass and args.exit_on_pass:
                print("PASS: all expected markers detected stably")
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
