#!/usr/bin/env python3
"""Run a quick ONNX YOLO detection test on a video file."""

import argparse
import collections
from pathlib import Path
import time

import cv2
try:
    from ultralytics import YOLO
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: ultralytics. Install it before running this script."
    ) from exc


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run an ONNX YOLO model on a video and save an annotated result."
    )
    parser.add_argument(
        "--model",
        default="fish_detection/yolo26n-seg.onnx",
        help="Path to the ONNX model.",
    )
    parser.add_argument(
        "--video",
        default="fish_detection/detected_fish.mp4",
        help="Path to the input video.",
    )
    parser.add_argument(
        "--output",
        default="fish_detection/onnx_test_output.mp4",
        help="Path for the annotated output video.",
    )
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold.")
    parser.add_argument("--iou", type=float, default=0.7, help="NMS IoU threshold.")
    parser.add_argument("--imgsz", type=int, default=640, help="Inference image size.")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Optional frame limit. Use 0 to process the whole video.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the annotated video while processing.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    package_root = Path(__file__).resolve().parents[1]
    model_path = resolve_path(args.model, package_root)
    video_path = resolve_path(args.video, package_root)
    output_path = Path(args.output).expanduser()

    if not model_path.exists():
        raise FileNotFoundError(f"Model not found: {model_path}")
    if not video_path.exists():
        raise FileNotFoundError(f"Video not found: {video_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        import onnxruntime  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: onnxruntime. Install it with:\n"
            "  python3 -m pip install onnxruntime\n"
            "Then rerun this script."
        ) from exc

    model = YOLO(str(model_path), task="detect")

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )

    frame_count = 0
    total_detections = 0
    frames_with_detections = 0
    start_time = time.perf_counter()

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if args.max_frames > 0 and frame_count >= args.max_frames:
            break

        results = model.predict(
            frame,
            conf=args.conf,
            iou=args.iou,
            imgsz=args.imgsz,
            verbose=False,
        )
        result = results[0]
        result.names = collections.defaultdict(lambda: "fish")
        detection_count = 0 if result.boxes is None else len(result.boxes)
        total_detections += detection_count
        if detection_count > 0:
            frames_with_detections += 1

        annotated = result.plot()
        cv2.putText(
            annotated,
            f"detections: {detection_count}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        writer.write(annotated)

        frame_count += 1
        if frame_count % 30 == 0:
            print(
                f"processed={frame_count} "
                f"current_detections={detection_count} "
                f"avg_detections={total_detections / frame_count:.2f}"
            )

        if args.show:
            cv2.imshow("ONNX fish detection test", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    elapsed = max(time.perf_counter() - start_time, 1e-6)
    capture.release()
    writer.release()
    cv2.destroyAllWindows()

    print("Done")
    print(f"model: {model_path}")
    print(f"video: {video_path}")
    print(f"output: {output_path}")
    print(f"frames_processed: {frame_count}")
    print(f"frames_with_detections: {frames_with_detections}")
    print(f"total_detections: {total_detections}")
    print(f"avg_detections_per_frame: {total_detections / max(frame_count, 1):.2f}")
    print(f"processing_fps: {frame_count / elapsed:.2f}")


def resolve_path(path_text, package_root):
    path = Path(path_text).expanduser()
    if path.exists() or path.is_absolute():
        return path
    if len(path.parts) > 1 and path.parts[0] == package_root.name:
        candidate = package_root.parent / path
        if candidate.exists():
            return candidate
    candidate = package_root / path.name
    if candidate.exists():
        return candidate
    return path


if __name__ == "__main__":
    main()