from inference import InferencePipeline
import cv2
import numpy as np
from inference.core.interfaces.stream.sinks import render_boxes
from functools import partial
# Callback to render predictions on each frame
# def render_boxes(predictions, frames):
#     # predictions contain YOLO boxes, class, confidence
#     # for pred in predictions:
#         # x0, y0, x1, y1 = pred["x"], pred["y"], pred["x"]+pred["width"], pred["y"]+pred["height"]
#         # label = f"{pred['class_name']} {pred['confidence']:.2f}"
#         # cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)
#         # cv2.putText(frame, label, (int(x0), int(y0)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

#     # Show annotated frame
#     frame = frames[0]
#     frame = np.array(frame)
#     frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#     cv2.imshow("YOLOv8 Inference", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         return False  # stop pipeline
#     # print(predictions)
#     return True
# Initialize pipeline with webcam (index 0)

output_size = (640, 480)
video_sink = cv2.VideoWriter(
    "output.avi",
    cv2.VideoWriter_fourcc(*"MJPG"),
    25.0,
    output_size
)

def display_write(frame_data):
    video_sink.write(frame_data[1])
    # cv2.imshow("YOLOv8 Inference", frame_data[1])
    # frame = frame_data[1].to_ndarray(format="rgb24")
    # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    cv2.imshow("YOLOv8 Inference", frame_data[1])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False  # stops pipeline
    return True


on_prediction = partial(
            render_boxes,
            display_size=output_size,
            on_frame_rendered=lambda frame_data: display_write(frame_data)
)

pipeline = InferencePipeline.init(
    model_id="yolov8n-640",
    video_reference=0,          # webcam
    on_prediction=on_prediction
)

# Start inference
pipeline.start()
pipeline.join()
video_sink.release()
