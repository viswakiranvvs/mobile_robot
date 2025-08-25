from inference import InferencePipeline
import cv2

# Callback to render predictions on each frame
def render_boxes(predictions, frame):
    # predictions contain YOLO boxes, class, confidence
    for pred in predictions:
        x0, y0, x1, y1 = pred["x"], pred["y"], pred["x"]+pred["width"], pred["y"]+pred["height"]
        label = f"{pred['class_name']} {pred['confidence']:.2f}"
        cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)
        cv2.putText(frame, label, (int(x0), int(y0)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # Show annotated frame
    cv2.imshow("YOLOv8 Inference", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False  # stop pipeline

# Initialize pipeline with webcam (index 0)
pipeline = InferencePipeline.init(
    model_id="yolov8n-640",
    video_reference=0,          # webcam
    on_prediction=render_boxes
)

# Start inference
pipeline.start()
