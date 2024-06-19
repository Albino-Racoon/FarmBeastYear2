import os

import yaml

# Nastavite poti do vaših map z usposabljanjem in validacijo
base_path = "C:/Users/jasar/Desktop/yolo_maline"
train_images = os.path.join(base_path, "train/images")
train_labels = os.path.join(base_path, "train/labels")
val_images = os.path.join(base_path, "val/images")
val_labels = os.path.join(base_path, "val/labels")

# Preverite, ali mape obstajajo (če ne, jih ustvarite)
os.makedirs(train_images, exist_ok=True)
os.makedirs(train_labels, exist_ok=True)
os.makedirs(val_images, exist_ok=True)
os.makedirs(val_labels, exist_ok=True)

# Specifikacija YAML datoteke
data = {
    'train': train_images.replace('\\', '/'),  # zagotavljanje pravilne poti za YAML
    'val': val_images.replace('\\', '/'),
    'nc': 1,  # število razredov
    'names': ['malina']  # seznam imen razredov
}

# Ustvarjanje YAML datoteke
yaml_path = os.path.join(base_path, "maline_DB.yaml")
with open(yaml_path, 'w') as file:
    documents = yaml.dump(data, file)

print(f"YAML datoteka ustvarjena: {yaml_path}")
