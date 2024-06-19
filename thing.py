import os
import cv2
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import accuracy_score
from joblib import dump, load

# Naložite in pripravite podatke
def load_images_from_folder(folder):
    images = []
    labels = []
    total_files = len(os.listdir(folder))  # Število datotek v mapi
    processed_files = 0  # Za sledenje napredku

    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder, filename))
        if img is not None:
            img = cv2.resize(img, (64, 64))  # Spreminjanje velikosti na manjšo skupno dimenzijo
            images.append(img.flatten())  # Sploščite sliko
            labels.append(0 if 'Grass' in folder else 1)  # Preprosta dodelitev oznak
        processed_files += 1
        print(f'Obdelano {processed_files} od {total_files} slik v mapi {folder}.')  # Izboljšano sporočilo napredka

    return images, labels

# Naložite slike rož in trave
flowers_path = r"C:\Users\jasar\Desktop\FB_roze\organized_images\Flowers"
grass_path = r"C:\Users\jasar\Desktop\FB_roze\organized_images\Grass"

flowers_images, flowers_labels = load_images_from_folder(flowers_path)
grass_images, grass_labels = load_images_from_folder(grass_path)

# Združite podatke in oznake
X = np.array(flowers_images + grass_images)
y = np.array(flowers_labels + grass_labels)

# Razdelite podatke na učne in testne sklope
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.25, random_state=42)

# Ustvarite in usposobite KNN klasifikator
knn = KNeighborsClassifier(n_neighbors=3)

# Simulacija več epoh - ponavljajte usposabljanje (primer za iteracije)
for epoch in range(25):
    knn.fit(X_train, y_train)
    y_pred = knn.predict(X_test)
    accuracy = accuracy_score(y_test, y_pred)
    print(f'Epoch {epoch + 1}: Točnost modela = {accuracy:.2f}')

model_path = r"C:\Users\jasar\Desktop\FB_roze\model_knn.joblib"
dump(knn, model_path)
print(f"Model shranjen na: {model_path}")
