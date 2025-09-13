# lightweight_yolo.py
from ultralytics import YOLO
import numpy as np
import cv2
from typing import List, Dict, Tuple, Optional

class LightweightYolo:
    """
    Lightweight YOLO wrapper using ultralytics (yolov8n by default).
    Usage:
        yolo = LightweightYolo(model_path="yolov8n.pt", device="cpu", conf=0.25)
        detections = yolo.detect(cv_bgr_image)
    Returns:
        List[{
            "bbox": (x1, y1, x2, y2),
            "confidence": float,
            "class_id": int,
            "class_name": str
        }, ...]
    """

    def __init__(self,
                 model_path: str = "yolov8n.pt",
                 device: str = "cpu",   # "cpu" or "cuda:0"
                 conf: float = 0.25,
                 imgsz: int = 640):
        """
        model_path: path or name (e.g. "yolov8n.pt")
        device: "cpu" or "cuda:0"
        conf: confidence threshold (0-1)
        imgsz: inference image size (square)
        """
        self.model = YOLO(model_path)          # loads model (downloads if necessary)
        self.device = device
        self.conf = conf
        self.imgsz = imgsz

        # If available, move model to chosen device
        try:
            self.model.to(self.device)
        except Exception:
            # model.to can be optional depending on ultralytics version; ignore if fails
            pass

        # convenience mapping from class id -> name (model.names)
        # model.names is a dict or list depending on ultralytics version
        try:
            self.names = {int(k): v for k, v in self.model.model.names.items()}
        except Exception:
            # fallback
            try:
                self.names = {i: n for i, n in enumerate(self.model.names)}
            except Exception:
                self.names = {}

    def _ensure_bgr(self, image: np.ndarray) -> np.ndarray:
        """
        Accept common image formats:
         - BGR (OpenCV default) -> pass-through
         - RGB -> convert to BGR (if shape[2]==3 and dtype==uint8 but appears RGB)
        We assume user provides an OpenCV BGR image; this is a safe-guard.
        """
        if image is None:
            raise ValueError("input image is None")
        if not isinstance(image, np.ndarray):
            raise TypeError("input must be numpy.ndarray")
        if image.ndim == 2:
            # single-channel -> convert to 3-channel BGR
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.ndim == 3 and image.shape[2] == 3:
            # assume BGR (user most likely has BGR from cv2)
            return image
        raise ValueError(f"Unsupported image shape: {image.shape}")

    def detect(self, image: np.ndarray, return_image: bool = False
               ) -> Tuple[List[Dict], Optional[np.ndarray]]:
        """
        Run detection on a single BGR image (numpy array).
        return_image: if True, returns an annotated copy of the image as second return value.
        Returns: (detections, annotated_image_or_None)
        """
        img = self._ensure_bgr(image)
        # Ultralytics accepts BGR or RGB numpy arrays; we pass BGR and let library handle.
        # Call model.predict - keep verbose False to minimize prints
        results = self.model.predict(source=img,
                                     imgsz=self.imgsz,
                                     conf=self.conf,
                                     device=self.device,
                                     verbose=False)

        # results is a list (one element per image). We processed one image.
        if not results:
            return [], None

        res = results[0]

        detections = []
        annotated = img.copy() if return_image else None

        # Extract boxes, confidences, classes
        # API differences: boxes can be at res.boxes or res.boxes.xyxy etc.
        try:
            boxes = res.boxes  # ultralytics.v8 results object
            xyxy = boxes.xyxy.cpu().numpy()     # shape: (N,4)
            confs = boxes.conf.cpu().numpy()    # shape: (N,)
            clsids = boxes.cls.cpu().numpy().astype(int)  # shape: (N,)
        except Exception:
            # fallback to using res.boxes.data if older API
            try:
                data = res.boxes.data.cpu().numpy()
                # data columns generally: x1, y1, x2, y2, conf, cls
                xyxy = data[:, :4]
                confs = data[:, 4]
                clsids = data[:, 5].astype(int)
            except Exception:
                # no detections
                return [], annotated

        for (box, conf, cid) in zip(xyxy, confs, clsids):
            x1, y1, x2, y2 = [int(round(x)) for x in box.tolist()]
            cname = self.names.get(int(cid), str(int(cid)))
            detections.append({
                "bbox": (x1, y1, x2, y2),
                "confidence": float(conf),
                "class_id": int(cid),
                "class_name": cname
            })
            if return_image:
                # draw box + label
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{cname} {conf:.2f}"
                # put label background
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
                cv2.putText(annotated, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return detections, annotated
