import numpy as np
import cv2
import time
import os

try:
    import tflite_runtime.interpreter as tflite
    print("[INFO] Utilisation de tflite_runtime")
except ImportError:
    import tensorflow as tf
    tflite = tf.lite
    print("[INFO] Utilisation de tensorflow.lite")

# CONFIGURATION
MODEL_PATH = os.path.join(os.path.dirname(__file__), "TF-lite/yolo11n-seg_tf/yolo11n-seg_float16.tflite")
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


# ============= FONCTIONS DE POST-TRAITEMENT YOLO =============

def xywh2xyxy(x):
    """Convertit les boxes de format xywh vers xyxy"""
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2
    y[..., 1] = x[..., 1] - x[..., 3] / 2
    y[..., 2] = x[..., 0] + x[..., 2] / 2
    y[..., 3] = x[..., 1] + x[..., 3] / 2
    return y


def nms(boxes, scores, iou_threshold):
    """Non-Maximum Suppression"""
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
        iou = (w * h) / (areas[i] + areas[order[1:]] - w * h + 1e-6)
        order = order[(np.where(iou <= iou_threshold)[0]) + 1]

    return keep


def postprocess(outputs, orig_shape, conf_thresh, iou_thresh):
    """
    Post-traitement des sorties YOLO11-seg TFLite
    Les sorties TFLite peuvent avoir un format différent de ONNX
    """
    # Gérer différents formats de sortie possibles
    if len(outputs) >= 2:
        # Format avec proto masks séparé
        predictions = outputs[0]
        proto = outputs[1]
    else:
        predictions = outputs[0]
        proto = None

    # Assurer le bon format (batch, features, num_predictions) -> (num_predictions, features)
    predictions = np.squeeze(predictions)
    
    # Si format (features, num_predictions), transposer
    if predictions.shape[0] < predictions.shape[1]:
        predictions = predictions.T

    # Extraire boxes, scores, et coefficients de masque
    boxes = predictions[:, :4]
    scores_all = predictions[:, 4:84]  # 80 classes COCO
    mask_coeffs = predictions[:, 84:] if predictions.shape[1] > 84 else None

    # Obtenir classe et score max pour chaque prédiction
    class_ids = np.argmax(scores_all, axis=1)
    scores = np.max(scores_all, axis=1)

    # Filtrer par seuil de confiance
    mask = scores > conf_thresh
    boxes, scores, class_ids = boxes[mask], scores[mask], class_ids[mask]
    if mask_coeffs is not None:
        mask_coeffs = mask_coeffs[mask]

    if len(boxes) == 0:
        return [], [], [], []

    # Convertir xywh -> xyxy
    boxes = xywh2xyxy(boxes)

    # Redimensionner les boxes vers la taille originale
    h_orig, w_orig = orig_shape[:2]
    boxes[:, [0, 2]] *= w_orig / INPUT_SIZE
    boxes[:, [1, 3]] *= h_orig / INPUT_SIZE

    # Appliquer NMS
    keep = nms(boxes, scores, iou_thresh)
    boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]

    # Traiter les masques de segmentation
    masks = []
    if proto is not None and mask_coeffs is not None:
        mask_coeffs = mask_coeffs[keep]
        proto = np.squeeze(proto)
        
        # Gérer différents formats de proto: (H, W, C) ou (C, H, W)
        if proto.shape[-1] == 32:  # Format (H, W, 32) - TFLite
            proto = np.transpose(proto, (2, 0, 1))  # -> (32, H, W)

        for i, coeffs in enumerate(mask_coeffs):
            # Combiner proto masks avec coefficients
            m = np.sum(proto * coeffs[:, None, None], axis=0)
            # Sigmoid
            m = 1 / (1 + np.exp(-m))
            # Redimensionner vers taille originale
            m = cv2.resize(m, (w_orig, h_orig))

            # Créer masque final dans la bounding box
            x1, y1, x2, y2 = boxes[i].astype(int)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_orig, x2), min(h_orig, y2)

            final_mask = np.zeros((h_orig, w_orig), dtype=np.uint8)
            final_mask[y1:y2, x1:x2] = (m[y1:y2, x1:x2] > 0.5).astype(np.uint8) * 255
            masks.append(final_mask)

    return boxes, scores, class_ids, masks


def draw_results(image, boxes, scores, class_ids, masks):
    """Dessiner les résultats de détection sur l'image"""
    overlay = image.copy()

    # Dessiner les masques de segmentation
    for i, mask in enumerate(masks):
        color = COLORS[class_ids[i]].tolist()
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0] = color
        overlay = cv2.addWeighted(overlay, 1, colored_mask, 0.4, 0)

    # Dessiner les bounding boxes et labels
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


def preprocess_frame(frame):
    """
    Prétraitement de l'image pour TFLite YOLO
    """
    # Redimensionner à la taille d'entrée du modèle
    resized = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    
    # Convertir BGR -> RGB
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    
    # Normaliser [0, 255] -> [0, 1]
    normalized = rgb.astype(np.float32) / 255.0
    
    # Ajouter dimension batch: (H, W, C) -> (1, H, W, C)
    batched = np.expand_dims(normalized, axis=0)
    
    return batched


# ============= MAIN =============

def main():
    print("=" * 60)
    print("YOLO11 Segmentation + TensorFlow Lite + RealSense")
    print("=" * 60)

    # Vérifier que le modèle existe
    if not os.path.exists(MODEL_PATH):
        print(f"[ERREUR] Modèle non trouvé: {MODEL_PATH}")
        print("Vérifiez le chemin du fichier .tflite")
        return

    # -------- Charger le modèle TFLite --------
    print(f"\n[INFO] Chargement du modèle TFLite: {MODEL_PATH}")
    
    try:
        # Créer l'interpréteur TFLite
        interpreter = tflite.Interpreter(model_path=MODEL_PATH)
        interpreter.allocate_tensors()
        
        # Obtenir les détails d'entrée/sortie
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        
        print(f"[INFO] Entrée: {input_details[0]['shape']} - {input_details[0]['dtype']}")
        for i, out in enumerate(output_details):
            print(f"[INFO] Sortie {i}: {out['shape']} - {out['dtype']}")
            
    except Exception as e:
        print(f"[ERREUR] Impossible de charger le modèle: {e}")
        return

    # -------- Ouvrir la caméra RealSense --------
    print(f"\n[INFO] Ouverture caméra RealSense (index {CAMERA_INDEX})...")
    
    # Option 1: Via OpenCV (plus simple)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    if not cap.isOpened():
        print("[ERREUR] Impossible d'ouvrir la caméra!")
        print("\nEssayez de changer CAMERA_INDEX (0, 1, 2...)")
        print("Listez les caméras avec: ls /dev/video* (Linux)")
        return

    print("[INFO] Caméra ouverte avec succès!")
    print("[INFO] Appuyez sur Ctrl+C pour quitter")
    print("-" * 60)

    # Variables pour FPS
    fps_counter, fps_start, fps_display = 0, time.time(), 0
    inference_times = []

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERREUR] Impossible de lire la frame")
                break

            # -------- Prétraitement --------
            input_data = preprocess_frame(frame)

            # -------- Inférence TFLite --------
            t_start = time.time()
            
            interpreter.set_tensor(input_details[0]['index'], input_data)
            interpreter.invoke()
            
            # Récupérer les sorties
            outputs = []
            for out in output_details:
                outputs.append(interpreter.get_tensor(out['index']))
            
            inference_time = (time.time() - t_start) * 1000
            inference_times.append(inference_time)

            # -------- Post-traitement --------
            boxes, scores, class_ids, masks = postprocess(
                outputs, frame.shape,
                CONF_THRESHOLD, IOU_THRESHOLD
            )

            # -------- Dessiner les résultats --------
            if len(boxes) > 0:
                annotated = draw_results(frame, boxes, scores, class_ids, masks)
            else:
                annotated = frame.copy()

            # -------- Calculer FPS --------
            fps_counter += 1
            if time.time() - fps_start >= 1.0:
                fps_display = fps_counter
                fps_counter, fps_start = 0, time.time()

            # -------- Afficher les infos --------
            avg_inference = np.mean(inference_times[-30:]) if inference_times else 0
            
            cv2.putText(annotated, f"FPS: {fps_display}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(annotated, f"Inference: {avg_inference:.1f}ms", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(annotated, f"Detections: {len(boxes)}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(annotated, "TFLite Float16", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            cv2.imshow("YOLO11-seg + TFLite + RealSense", annotated)
            cv2.waitKey(1)

    except KeyboardInterrupt:
        print("\n[INFO] Interruption par l'utilisateur")

    finally:
        # Nettoyage
        cap.release()
        cv2.destroyAllWindows()
        
        # Statistiques finales
        if inference_times:
            print("\n" + "=" * 60)
            print("STATISTIQUES")
            print("=" * 60)
            print(f"Temps d'inférence moyen: {np.mean(inference_times):.2f} ms")
            print(f"Temps d'inférence min: {np.min(inference_times):.2f} ms")
            print(f"Temps d'inférence max: {np.max(inference_times):.2f} ms")
        
        print("[INFO] Terminé")


if __name__ == "__main__":
    main()
