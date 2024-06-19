from ultralytics import YOLO
import time


tic = time.time()
model = YOLO("yolov8n.pt")

model.train(data=r"C:\Users\jasar\Desktop\yolo_maline\maline_DB.yaml", epochs=5, imgsz=640)
metrics = model.val()

toc = time.time()-tic
print(toc)

from sklearn.model_selection import GridSearchCV
import time
from ultralytics import YOLO

# Definirajte razpon parametrov, ki jih želite preizkusiti
param_grid = {'epochs': [5, 10, 15,20,25,30], 'imgsz': [416, 512, 640]}

# Ustvarite objekt modela YOLO
model = YOLO("yolov8n.pt")

# Ustvarite objekt GridSearchCV za prečno preverjanje
grid_search = GridSearchCV(estimator=model, param_grid=param_grid, cv=5)

# Začnite merjenje časa
tic = time.time()

# Zaženite prečno preverjanje
grid_search.fit(data="maline_DB.yaml")

# Prenehajte merjenje časa
toc = time.time() - tic

# Izpišite najboljše rezultate in čas izvajanja
print("Najboljši rezultat:", grid_search.best_params_)
print("Čas izvajanja:", toc)