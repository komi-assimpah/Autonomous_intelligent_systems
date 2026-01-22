"""
Serveur de streaming RealSense + YOLO11 Segmentation pour Jetson
- Capture via OpenCV
- Inference ONNX YOLO11 segmentation ou TFLite
- Envoi des frames annotées en JPEG via TCP

Usage: python realsense_server_jetson.py --model chemin/vers/modele.onnx
       python realsense_server_jetson.py --model chemin/vers/modele.tflite
"""
import socket
import struct
import pickle
import io
import numpy as np
import cv2
import pyrealsense2 as rs
import time
import os
import argparse
import onnxruntime as ort
import psutil
import statistics
from collections import deque
from datetime import datetime
from typing import Optional, Tuple
import csv

try:
    import tflite_runtime.interpreter as tflite
    TFLITE_AVAILABLE = True
except ImportError:
    TFLITE_AVAILABLE = False

# Modèles candidats (préférence pour l'optimisé, fallback vers non-optimisé)
_HERE = os.path.dirname(__file__)
_HOME = os.path.expanduser("~")
MODEL_CANDIDATES = [
    os.path.join(_HERE, "yolo11n-seg.optimized.onnx"),
    os.path.join(_HERE, "yolo11n-seg.onnx"),
    os.path.join(_HOME, "yolo11n-seg.optimized.onnx"),
    os.path.join(_HOME, "yolo11n-seg.onnx"),
]

# Permettre de forcer un modèle précis via variable d'env
MODEL_PATH_FORCE = os.environ.get("MODEL_PATH_FORCE")
if MODEL_PATH_FORCE:
    # Insérer en tête si le chemin est fourni
    MODEL_CANDIDATES = [MODEL_PATH_FORCE] + [p for p in MODEL_CANDIDATES if p != MODEL_PATH_FORCE]

WIDTH = 640
HEIGHT = 480
FPS = 30
INPUT_SIZE = 640
CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.45
CAMERA_INDEX = 2
HOST = "0.0.0.0"
PORT = 8485
JPEG_QUALITY = 80

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
            try:
                m = np.sum(proto * coeffs[:, None, None], axis=0)
                m = 1 / (1 + np.exp(-m))
                m = cv2.resize(m, (w_orig, h_orig))
                x1, y1, x2, y2 = boxes[i].astype(int)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w_orig, x2), min(h_orig, y2)
                final_mask = np.zeros((h_orig, w_orig), dtype=np.uint8)
                final_mask[y1:y2, x1:x2] = (m[y1:y2, x1:x2] > 0.5).astype(np.uint8) * 255
                masks.append(final_mask)
            except Exception as e:
                # Si la génération de masque échoue, créer un masque vide
                final_mask = np.zeros((h_orig, w_orig), dtype=np.uint8)
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
        cv2.putText(overlay, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return overlay

def setup_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    pipeline.start(config)
    # Laisser quelques frames de chauffe pour stabiliser l'expo/auto-white-balance
    for _ in range(5):
        pipeline.wait_for_frames()
    return pipeline


def setup_model(model_path: str):
    """Charge le modèle ONNX ou TFLite spécifié."""
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Modèle non trouvé: {model_path}")
    
    print(f"[INFO] Chargement du modèle: {model_path}")
    
    # Déterminer le format
    if model_path.endswith('.tflite'):
        return setup_tflite_model(model_path)
    else:
        return setup_onnx_model(model_path)

def setup_tflite_model(model_path: str) -> Tuple:
    """Charge un modèle TFLite."""
    if not TFLITE_AVAILABLE:
        raise ImportError("tflite_runtime n'est pas installé. Installez-le avec: pip install tflite-runtime")
    
    print("[INFO] Chargement TFLite...")
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    
    print("[INFO] Providers actifs: [TFLiteRuntime]")
    
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    return ('tflite', interpreter, input_details, output_details)

def setup_onnx_model(model_path: str):
    """Charge un modèle ONNX avec gestion spéciale pour ARM."""
    available = ort.get_available_providers()
    print(f"[INFO] Providers disponibles: {available}")
    
    # Sur ARM (Raspberry Pi), forcer CPUExecutionProvider pour la stabilité
    providers = ["CPUExecutionProvider"]
    
    try:
        print(f"[INFO] Création de la session ONNX avec providers: {providers}")
        
        # Options de session pour réduire l'usage mémoire
        sess_opts = ort.SessionOptions()
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_BASIC
        sess_opts.intra_op_num_threads = 2  # Limiter les threads pour Raspberry Pi
        sess_opts.inter_op_num_threads = 1
        
        session = ort.InferenceSession(
            model_path, 
            providers=providers,
            sess_options=sess_opts
        )
        print(f"[INFO] Providers actifs: {session.get_providers()}")
    except Exception as e:
        print(f"[ERROR] Impossible de créer la session ONNX: {e}")
        raise
    
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]
    return ('onnx', session, input_name, output_names)


def start_server(model_path: str, benchmark_frames: Optional[int] = None, csv_path: Optional[str] = None, max_seconds: Optional[int] = None):
    model_info = setup_model(model_path)
    model_type = model_info[0]
    pipeline = setup_camera()
    cv2.setUseOptimized(True)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Serveur en écoute sur {HOST}:{PORT}")

    # Toujours activer le CSV pour envoi via TCP
    import io
    csv_buffer = io.StringIO()
    writer = csv.writer(csv_buffer)
    writer.writerow([
        "timestamp", "frame", "model", "pre_ms", "infer_ms", "post_ms",
        "fps", "cpu_percent", "rss_mb", "detections"
    ])

    client_socket = None
    try:
        client_socket, addr = server_socket.accept()
        print(f"Client connecté: {addr}")

        fps_counter, fps_start, fps_display = 0, time.time(), 0
        proc = psutil.Process(os.getpid())
        proc.cpu_percent(None)
        frame_idx = 0
        
        # Timer pour forcer la sortie après une durée maximale (ou respecter benchmark_frames)
        loop_start_time = time.time()
        loop_timeout = max_seconds if (isinstance(max_seconds, int) and max_seconds > 0) else 35

        while True:
            # Vérifier le timeout de la boucle
            if benchmark_frames is None and time.time() - loop_start_time >= loop_timeout:
                print(f"[INFO] Timeout boucle atteint ({loop_timeout}s) - arrêt")
                break
            
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())

            if model_type == 'tflite':
                t0 = time.perf_counter()
                # TFLite attend un tenseur NHWC float32 normalisé
                input_data = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
                input_data = input_data.astype(np.float32) / 255.0
                input_data = np.expand_dims(input_data, axis=0)
                pre_ms = (time.perf_counter() - t0) * 1000

                t1 = time.perf_counter()
                _, interpreter, input_details, output_details = model_info
                interpreter.set_tensor(input_details[0]['index'], input_data)
                interpreter.invoke()
                outputs = [interpreter.get_tensor(od['index']) for od in output_details]
                infer_ms = (time.perf_counter() - t1) * 1000
            else:
                try:
                    t0 = time.perf_counter()
                    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (INPUT_SIZE, INPUT_SIZE), swapRB=True, crop=False)
                    pre_ms = (time.perf_counter() - t0) * 1000

                    t1 = time.perf_counter()
                    _, session, input_name, output_names = model_info
                    outputs = session.run(output_names, {input_name: blob})
                    infer_ms = (time.perf_counter() - t1) * 1000
                except RuntimeError as e:
                    # ONNX Runtime peut avoir des problèmes d'allocation sur ARM - sauter ce frame
                    print(f"[WARNING] Erreur ONNX au frame {frame_idx} (mémoire insuffisante?): {str(e)[:100]}")
                    continue  # Skip this frame and continue
                except Exception as e:
                    # Autres erreurs - log et continue
                    print(f"[WARNING] Erreur inférence ONNX au frame {frame_idx}: {str(e)[:100]}")
                    continue

            t2 = time.perf_counter()
            try:
                boxes, scores, class_ids, masks = postprocess(outputs, frame.shape, CONF_THRESHOLD, IOU_THRESHOLD)
                annotated = draw_results(frame, boxes, scores, class_ids, masks) if len(boxes) > 0 else frame
            except Exception as e:
                print(f"[WARNING] Erreur postprocessing au frame {frame_idx}: {str(e)[:100]}")
                boxes, scores, class_ids, masks = [], [], [], []
                annotated = frame
            post_ms = (time.perf_counter() - t2) * 1000

            fps_counter += 1
            if time.time() - fps_start >= 1.0:
                fps_display = fps_counter
                fps_counter, fps_start = 0, time.time()

            cpu_now = proc.cpu_percent(None)
            rss_mb = proc.memory_info().rss / (1024*1024)

            if writer:
                writer.writerow([
                    datetime.utcnow().isoformat(),
                    frame_idx,
                    os.path.basename(model_path),
                    f"{pre_ms:.2f}",
                    f"{infer_ms:.2f}",
                    f"{post_ms:.2f}",
                    fps_display,
                    f"{cpu_now:.1f}",
                    f"{rss_mb:.1f}",
                    len(boxes),
                ])

            cv2.putText(annotated, f"FPS: {fps_display}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(annotated, f"CPU: {cpu_now:.1f}%  MEM: {rss_mb:.1f}MB", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated, f"Detections: {len(boxes)}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            ok, buffer = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                continue

            payload = pickle.dumps(buffer)
            header = struct.pack('!I', len(payload))
            try:
                client_socket.sendall(header + payload)
            except (BrokenPipeError, ConnectionResetError):
                print("Client déconnecté")
                break

            frame_idx += 1
            if benchmark_frames and frame_idx >= benchmark_frames:
                print(f"Arrêt après {frame_idx} frames (mode benchmark)")
                break

        # Envoyer le CSV au client IMMEDIATEMENT après la boucle
        print(f"[DEBUG] Sortie boucle - envoi CSV")
        if writer and csv_buffer and client_socket:
            csv_content = csv_buffer.getvalue()
            csv_bytes = csv_content.encode('utf-8')
            print(f"[DEBUG] CSV content length: {len(csv_bytes)} bytes")
            
            marker = b"CSV:"
            size_header = struct.pack('!I', len(csv_bytes))
            try:
                client_socket.sendall(marker + size_header + csv_bytes)
                print(f"[INFO] CSV envoyé au client ({len(csv_bytes)} bytes)")
            except (BrokenPipeError, ConnectionResetError) as e:
                print(f"[ERROR] Impossible d'envoyer CSV: {e}")

    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        
        try:
            if client_socket:
                client_socket.close()
        except Exception:
            pass
        
        server_socket.close()
        print("Serveur arrêté")


def main():
    parser = argparse.ArgumentParser(description="Serveur YOLO11 Segmentation pour Jetson")
    parser.add_argument(
        "--model", "-m",
        type=str,
        required=True,
        help="Chemin vers le modèle ONNX (ex: yolo11n-seg.onnx)"
    )
    parser.add_argument(
        "--benchmark-frames",
        type=int,
        default=None,
        help="Nombre de frames à traiter puis arrêter (mode benchmark)"
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Chemin vers le fichier CSV de mesures (facultatif)"
    )
    parser.add_argument(
        "--max-seconds",
        type=int,
        default=None,
        help="Durée maximale (secondes) avant arrêt automatique"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("Serveur RealSense + YOLO11 Segmentation")
    print("=" * 60)
    print(f"Modèle: {args.model}")
    if args.benchmark_frames:
        print(f"Frames benchmark: {args.benchmark_frames}")
    if args.csv:
        print(f"CSV: {args.csv}")
    print("=" * 60)

    start_server(args.model, benchmark_frames=args.benchmark_frames, csv_path=args.csv, max_seconds=args.max_seconds)


if __name__ == "__main__":
    main()
