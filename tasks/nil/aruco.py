import argparse
import cv2
import numpy as np


def create_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    parameters.adaptiveThreshConstant = 10
    parameters.minMarkerPerimeterRate = 0.01
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.perspectiveRemovePixelPerCell = 8
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    return detector, parameters


def extract_aruco_positions(corners, ids, frame_height):
    positions = {}
    lowest_marker = None

    if ids is None or len(ids) == 0:
        return positions, lowest_marker

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        points = marker_corners.reshape(-1, 2)
        center_x = int(np.mean(points[:, 0]))
        center_y = int(np.mean(points[:, 1]))
        bottom_y = int(np.max(points[:, 1]))
        bottom_points = points[points[:, 1] >= bottom_y - 1]
        bottom_x = int(np.mean(bottom_points[:, 0])) if bottom_points.size else center_x

        width = np.linalg.norm(points[0] - points[1])
        height = np.linalg.norm(points[1] - points[2])
        marker_size = int((width + height) / 2)
        y_offset = int(marker_size * 0.85)
        adjusted_y = min(bottom_y + y_offset, frame_height - 1)

        positions[int(marker_id)] = {
            "x": center_x,
            "y": adjusted_y,
            "center_x": center_x,
            "center_y": center_y,
            "bottom_x": bottom_x,
            "bottom_y": bottom_y,
            "adjusted_y": adjusted_y,
            "size": marker_size,
        }

    if positions:
        lowest_id = max(positions.keys(), key=lambda key: positions[key]["bottom_y"])
        lowest_marker = {"id": lowest_id, **positions[lowest_id]}

    return positions, lowest_marker


def detect_aruco_on_stream(stream_url: str, debug: bool = False) -> None:
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open stream: {stream_url}")

    detector, parameters = create_aruco_detector()

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to read frame from stream. Exiting.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(gray)
        positions, lowest = extract_aruco_positions(corners, ids, frame.shape[0])

        mask = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY,
            23,
            parameters.adaptiveThreshConstant,
        )

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            print("Aruco positions:", positions)
            if lowest is not None:
                print("Lowest marker:", lowest)

            for marker_id, info in positions.items():
                cv2.circle(frame, (info["x"], info["y"]), 4, (0, 255, 0), -1)
                cv2.putText(
                    frame,
                    f"ID {marker_id}: ({info['x']},{info['y']})",
                    (info["x"] + 5, info["y"] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            if lowest is not None:
                cv2.putText(
                    frame,
                    f"Lowest: id={lowest['id']} x={lowest['x']} y={lowest['y']}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.circle(frame, (lowest["x"], lowest["y"]), 8, (0, 255, 255), 2)
        elif debug and rejected is not None and len(rejected) > 0:
            cv2.aruco.drawDetectedMarkers(frame, rejected, borderColor=(0, 0, 255))

        cv2.imshow("Robot Camera Stream", frame)
        cv2.imshow("Aruco Tuning Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect ArUco markers from a robot MJPEG stream.")
    parser.add_argument(
        "--stream",
        default="http://10.197.219.91:7123/stream.mjpg",
        help="URL of the MJPEG stream (default: robot stream)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Show rejected marker candidates as red boxes for tuning only.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        detect_aruco_on_stream(args.stream, args.debug)
    except Exception as exc:
        print(f"Error: {exc}")
        cv2.destroyAllWindows()
