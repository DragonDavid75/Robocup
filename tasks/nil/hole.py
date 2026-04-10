import argparse
import math

import cv2
import numpy as np


def create_hole_detector_parameters():
    return {
        "brown_lower": np.array([5, 80, 50], dtype=np.uint8),
        "brown_upper": np.array([30, 255, 255], dtype=np.uint8),
        "brown_min_area_ratio": 0.0005,
        "brown_max_area_ratio": 0.1,
        "brown_min_circularity": 0.2,
        "brown_morph_kernel": (7, 7),
    }


def detect_brown_hole(frame: np.ndarray, params: dict) -> tuple[bool, tuple[int, int], int, np.ndarray]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    brown_mask = cv2.inRange(hsv, params["brown_lower"], params["brown_upper"])

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, params["brown_morph_kernel"])
    brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(brown_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = frame.shape[:2]
    frame_area = width * height

    best_candidate = None
    best_score = 0.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area <= 0:
            continue

        area_ratio = area / frame_area
        if area_ratio < params["brown_min_area_ratio"] or area_ratio > params["brown_max_area_ratio"]:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue

        circularity = 4 * math.pi * area / (perimeter * perimeter)
        if circularity < params["brown_min_circularity"]:
            continue

        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if radius < 5:
            continue

        score = circularity * area_ratio
        if score > best_score:
            best_score = score
            best_candidate = ((int(x), int(y)), int(radius), circularity, area_ratio)

    return (False, (0, 0), 0, brown_mask) if best_candidate is None else (True, best_candidate[0], best_candidate[1], brown_mask)


def detect_hole_on_stream(stream_url: str) -> None:
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open stream: {stream_url}")

    params = create_hole_detector_parameters()

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to read frame from stream. Exiting.")
            break

        height, width = frame.shape[:2]
        crop_w = int(width * 3 / 4)
        crop_h = int(height * 1 / 2)
        x0 = (width - crop_w) // 2
        y0 = (height - crop_h) // 2
        crop = frame[y0 : y0 + crop_h, x0 : x0 + crop_w]

        found, center, radius, brown_mask = detect_brown_hole(crop, params)

        display = crop.copy()
        cv2.rectangle(display, (0, 0), (crop_w - 1, crop_h - 1), (255, 0, 0), 2)

        if found:
            cv2.circle(display, center, radius, (0, 255, 0), 2)
            cv2.putText(
                display,
                f"Brown hole: {center} r={radius}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            print(f"Detected brown hole at {center} with radius {radius}")
        else:
            cv2.putText(
                display,
                "No brown hole detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow("Robot Camera Stream", display)
        cv2.imshow("Brown Hole Mask", brown_mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect a brown hole in the central cropped region of the robot camera stream.")
    parser.add_argument(
        "--stream",
        default="http://10.197.219.91:7123/stream.mjpg",
        help="URL of the MJPEG stream to analyze.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        detect_hole_on_stream(args.stream)
    except Exception as exc:
        print(f"Error: {exc}")
        cv2.destroyAllWindows()
