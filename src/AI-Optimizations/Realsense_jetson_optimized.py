import numpy as np
import cv2
import time
import os
import onnxruntime as ort

# CONFIGURATION
MODEL_PATH = os.path.join(os.path.dirname(__file__), "yolo11n-seg.optimized.onnx")   # modèle optimisé
WIDTH = 640
HEIGHT = 480
FPS = 30
INPUT_SIZE = 640
CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.45
CAMERA_INDEX = 0

# Classes COCO
CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(len(CLASSES), 3), dtype=np.uint8)

# YOLO POST-PROCESSING
def xywh2xyxy(x):
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2
    y[..., 1] = x[..., 1] - x[..., 3] / 2
    y[..., 2] = x[..., 0] + x[..., 2] / 2
    y[..., 3] = x[..., 1] + x[..., 3] / 2
    return y

def nms(boxes, scores, iou_threshold):
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        iou = (w * h) / (areas[i] + areas[order[1:]] - w * h)
        order = order[(np.where(iou <= iou_threshold)[0]) + 1]

    return keep

def postprocess(outputs, orig_shape, conf_thresh, iou_thresh):
    predictions = outputs[0]
    proto = outputs[1] if len(outputs) > 1 else None

    predictions = np.squeeze(predictions).T
    boxes = predictions[:, :4]
    scores_all = predictions[:, 4:84]
    mask_coeffs = predictions[:, 84:] if predictions.shape[1] > 84 else None

    class_ids = np.argmax(scores_all, axis=1)
    scores = np.max(scores_all, axis=1)

    mask = scores > conf_thresh
    boxes, scores, class_ids = boxes[mask], scores[mask], class_ids[mask]
    if mask_coeffs is not None:
        mask_coeffs = mask_coeffs[mask]

    if len(boxes) == 0:
        return [], [], [], []

    boxes = xywh2xyxy(boxes)

    h_orig, w_orig = orig_shape[:2]
    boxes[:, [0, 2]] *= w_orig / INPUT_SIZE
    boxes[:, [1, 3]] *= h_orig / INPUT_SIZE

    keep = nms(boxes, scores, iou_thresh)
    boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]

    masks = []
    if proto is not None and mask_coeffs is not None:
        mask_coeffs = mask_coeffs[keep]
        proto = np.squeeze(proto)

        for i, coeffs in enumerate(mask_coeffs):
            m = np.sum(proto * coeffs[:, None, None], axis=0)
            m = 1 / (1 + np.exp(-m))
            m = cv2.resize(m, (w_orig, h_orig))

            x1, y1, x2, y2 = boxes[i].astype(int)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_orig, x2), min(h_orig, y2)

            final_mask = np.zeros((h_orig, w_orig), dtype=np.uint8)
            final_mask[y1:y2, x1:x2] = (m[y1:y2, x1:x2] > 0.5).astype(np.uint8) * 255
            masks.append(final_mask)

    return boxes, scores, class_ids, masks

def draw_results(image, boxes, scores, class_ids, masks):
    overlay = image.copy()

    for i, mask in enumerate(masks):
        color = COLORS[class_ids[i]].tolist()
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0] = color
        overlay = cv2.addWeighted(overlay, 1, colored_mask, 0.4, 0)

    for i, box in enumerate(boxes):
        x1, y1, x2, y2 = box.astype(int)
        color = COLORS[class_ids[i]].tolist()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)

        label = f"{CLASSES[class_ids[i]]}: {scores[i]:.2f}"
        (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(overlay, (x1, y1 - 20), (x1 + w, y1), color, -1)
        cv2.putText(overlay, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return overlay

# MAIN
def main():
    print("=" * 60)
    print("YOLO11 Segmentation + ONNX Runtime (Optimized)")
    print("=" * 60)

    # -------- ONNX Runtime Session --------
    available = ort.get_available_providers()
    providers = []

    if "TensorrtExecutionProvider" in available:
        providers.append("TensorrtExecutionProvider")
    if "CUDAExecutionProvider" in available:
        providers.append("CUDAExecutionProvider")
    providers.append("CPUExecutionProvider")

    print("[INFO] ONNX Runtime providers:", providers)

    session = ort.InferenceSession(MODEL_PATH, providers=providers)
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]

    # -------- Camera --------
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    if not cap.isOpened():
        print("[ERREUR] Impossible d'ouvrir la caméra")
        return

    print("[INFO] Caméra ouverte!")
    print("[INFO] Ctrl+C pour quitter")
    print("-" * 60)

    fps_counter, fps_start, fps_display = 0, time.time(), 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            blob = cv2.dnn.blobFromImage(
                frame, 1/255.0,
                (INPUT_SIZE, INPUT_SIZE),
                swapRB=True, crop=False
            )

            outputs = session.run(
                output_names,
                {input_name: blob}
            )

            boxes, scores, class_ids, masks = postprocess(
                outputs, frame.shape,
                CONF_THRESHOLD, IOU_THRESHOLD
            )

            annotated = draw_results(frame, boxes, scores, class_ids, masks) \
                if len(boxes) > 0 else frame

            fps_counter += 1
            if time.time() - fps_start >= 1.0:
                fps_display = fps_counter
                fps_counter, fps_start = 0, time.time()

            cv2.putText(annotated, f"FPS: {fps_display}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("YOLO + ONNX Runtime (Optimized)", annotated)
            cv2.waitKey(1)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Terminé")

if __name__ == "__main__":
    main()
