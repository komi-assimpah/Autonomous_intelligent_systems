import time
import statistics
import numpy as np
import onnxruntime as ort
from pathlib import Path

MODEL_IN = Path("yolo11n-seg.onnx")
MODEL_OUT = Path("yolo11n-seg.optimized.onnx")

def pick_providers():
    """
    Choisit le meilleur provider disponible
    """
    available = ort.get_available_providers()
    # Ordre "agressif": TensorRT > CUDA > CPU
    preferred = []
    if "TensorrtExecutionProvider" in available:
        preferred.append("TensorrtExecutionProvider")
    if "CUDAExecutionProvider" in available:
        preferred.append("CUDAExecutionProvider")
    preferred.append("CPUExecutionProvider")
    return preferred, available

def print_io(session: ort.InferenceSession):
    print("\n=== MODEL I/O ===")
    print("Inputs:")
    for i in session.get_inputs():
        print(f" - name={i.name}  shape={i.shape}  type={i.type}")
    print("Outputs:")
    for o in session.get_outputs():
        print(f" - name={o.name}  shape={o.shape}  type={o.type}")
    print("=================\n")

def make_session(model_path: Path, providers):
    so = ort.SessionOptions()

    # Graph optimizations: Basic/Extended/Layout selon ORT
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

    # Export du modèle optimisé (ORT écrit un graphe optimisé)
    so.optimized_model_filepath = str(MODEL_OUT)
    so.intra_op_num_threads = 0  # 0 => ORT décide
    so.inter_op_num_threads = 0

    # Crée la session avec providers choisis
    return ort.InferenceSession(str(model_path), sess_options=so, providers=providers)

def benchmark(session: ort.InferenceSession, runs=50, warmup=10):
    # Construit un input dummy à partir du 1er input du modèle
    inp = session.get_inputs()[0]
    name = inp.name

    # Remplace les dimensions dynamiques par une valeur raisonnable (1)
    # Exemple typique YOLO: [1, 3, 640, 640]
    shape = []
    for d in inp.shape:
        if isinstance(d, int):
            shape.append(d)
        else:
            shape.append(1)

    # Si le modèle a une entrée image classique, on force 3 canaux + 640x640 si dynamiques
    if len(shape) == 4:
        # heuristique: NCHW
        if shape[1] == 1:
            shape[1] = 3
        if shape[2] == 1:
            shape[2] = 640
        if shape[3] == 1:
            shape[3] = 640

    x = (np.random.rand(*shape).astype(np.float32))

    # Warmup
    for _ in range(warmup):
        session.run(None, {name: x})

    # Timed runs
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        session.run(None, {name: x})
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)

    times_sorted = sorted(times)
    p50 = times_sorted[int(0.50 * (len(times_sorted) - 1))]
    p90 = times_sorted[int(0.90 * (len(times_sorted) - 1))]
    print(f"Runs={runs} | mean={statistics.mean(times):.2f} ms | p50={p50:.2f} ms | p90={p90:.2f} ms")

def main():
    if not MODEL_IN.exists():
        raise FileNotFoundError(f"Model not found: {MODEL_IN.resolve()}")

    providers, available = pick_providers()
    print("Available providers:", available)
    print("Using providers (in priority order):", providers)

    session = make_session(MODEL_IN, providers=providers)
    print_io(session)

    # Benchmark (sur input dummy)
    benchmark(session, runs=50, warmup=10)

    # Confirme export
    if MODEL_OUT.exists():
        print(f"\n Optimized model exported to: {MODEL_OUT.resolve()}")
    else:
        print("\n Optimized model was not exported. Check permissions / path.")

if __name__ == "__main__":
    main()
