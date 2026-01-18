import argparse
from pathlib import Path
import onnx

try:
    from onnxsim import simplify
except Exception as e:
    raise SystemExit(
        "onnx-simplifier n'est pas installé. Installez-le avec:\n"
        "  python -m pip install onnxsim\n"
        f"Erreur: {e}"
    )


def main():
    parser = argparse.ArgumentParser(description="Optimiser un modèle ONNX avec onnx-simplifier (sans NCHWc)")
    parser.add_argument("model_in", type=Path, help="Chemin du modèle d'entrée (.onnx)")
    parser.add_argument("--model-out", type=Path, default=None, help="Chemin du modèle de sortie (.onnx)")
    parser.add_argument("--input-shape", type=str, default="1,3,640,640", help="Forme d'entrée pour la simplification (ex: 1,3,640,640)")
    args = parser.parse_args()

    model_in: Path = args.model_in
    if args.model_out is None:
        model_out = model_in.with_suffix(".optimized.onnx")
    else:
        model_out = args.model_out

    if not model_in.exists():
        raise SystemExit(f"Modèle introuvable: {model_in}")

    print(f"[INFO] Chargement: {model_in}")
    model = onnx.load(model_in)

    shape = tuple(int(x) for x in args.input_shape.split(","))
    input_shapes = {}
    # Essaye d'inférer le nom du premier input
    if model.graph.input:
        input_name = model.graph.input[0].name
        input_shapes[input_name] = shape
        print(f"[INFO] Input '{input_name}' -> shape={shape}")

    print("[INFO] Simplification en cours (peut prendre du temps)...")
    simplified_model, check_ok = simplify(model, input_shapes=input_shapes)
    if not check_ok:
        print("[WARN] La vérification du modèle simplifié a échoué. Le modèle sera quand même sauvegardé.")

    onnx.save(simplified_model, model_out)
    print(f"[OK] Modèle optimisé sauvegardé: {model_out}")


if __name__ == "__main__":
    main()
