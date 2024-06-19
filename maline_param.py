import torch
from ultralytics import YOLO

# Poskusi inicializirati model
try:
    model = YOLO("yolov8n.pt")
    print("Model loaded successfully.")
except AttributeError as e:
    print(f"Failed to load model due to an attribute error: {e}")

# Tukaj dodaj preostanek svoje kodo za nadaljnje postopke


def evaluate_and_save_metrics(model, data_path, output_path):
    # Evaluacija modela na validacijskem sklopu
    results = model.evaluate(data_path)

    # Rezultati vsebujejo različne metrike, kot je mAP@.5
    map50 = results.metrics['map50']  # Preveri pravilno ime metrike

    # Shranjevanje metrik v JSON datoteko
    import json
    with open(output_path, 'w') as f:
        json.dump({'map50': map50}, f)

    print(f"Metrika map50 shranjena v {output_path}")


# Pot do validacijskih podatkov
data_path = r"C:\Users\jasar\Desktop\yolo_maline\train\maline_DB.yaml"


# Pot do izhodne datoteke za shranjevanje metrik
output_file_path = r"C:\Users\jasar\Desktop\yolo_maline\metrics.json"


# Evaluacija modela in shranjevanje metrik
evaluate_and_save_metrics(model, data_path, output_file_path)
