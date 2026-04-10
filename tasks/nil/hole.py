import argparse
import math

import cv2
import numpy as np


def create_hole_detector_parameters():
    return {
        "adaptive_blocksize": 51,
        "adaptive_C": 12,
        "min_area_ratio": 0.0005,
        "max_area_ratio": 0.05,
        "min_circularity": 0.4,
        "morph_kernel": (7, 7),
    }


def detect_floor_hole(frame: np.ndarray, params: dict) -> tuple[bool, tuple[int, int], int, np.ndarray]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    thresh = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        params["adaptive_blocksize"],
        params["adaptive_C"],
    )

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, params["morph_kernel"])
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = frame.shape[:2]
    frame_area = width * height

    best_candidate = None
    best_score = 0.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area <= 0:
            continue

        area_ratio = area / frame_area
        if area_ratio < params["min_area_ratio"] or area_ratio > params["max_area_ratio"]:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue

        circularity = 4 * math.pi * area / (perimeter * perimeter)
        if circularity < params["min_circularity"]:
            continue

        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if radius < 5:
            continue

        score = circularity * area_ratio
        if score > best_score:
            best_score = score
            best_candidate = ((int(x), int(y)), int(radius), circularity, area_ratio)

    if best_candidate is None:
        return False, (0, 0), 0, thresh

    center, radius, circularity, area_ratio = best_candidate
    return True, center, radius, thresh


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

        found, center, radius, mask = detect_floor_hole(frame, params)

        display = frame.copy()
        if found:
            cv2.circle(display, center, radius, (0, 255, 0), 2)
            cv2.putText(
                display,
                f"Hole detected: {center} r={radius}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            print(f"Detected hole at {center} with radius {radius}")
        else:
            cv2.putText(
                display,
                "No hole detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow("Robot Camera Stream", display)
        cv2.imshow("Hole Detection Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect a small dark hole in the floor from the robot camera stream.")
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
