import os
from yolov5 import train  # Uvozi funkcijo train iz lokalne namestitve YOLOv5

# Poti do direktorijev in datotek
base_dir = "C:/Users/jasar/Desktop/FB_roze_yolo"
data_yaml_path = os.path.join(base_dir, "myDataSet", "flow.yaml")
weights_path = "yolov5s.pt"  # Spremenite, če imate drugačne začetne teže

# Parametri za usposabljanje
train_params = {
    'img': 640,  # velikost slike
    'batch': 16,  # velikost paketa
    'epochs': 50,  # število epoh
    'data': data_yaml_path,  # pot do YAML datoteke z nastavitvami dataset-a
    'weights': weights_path,  # začetne teže modela
    'device': 'cpu'  # Nastavi na 'cpu' za usposabljanje na CPU
}

# Funkcija za zagon usposabljanja
def train_yolov5(params):
    # Zagon usposabljanja
    train.run(**params)

# Zagon usposabljanja
train_yolov5(train_params)
