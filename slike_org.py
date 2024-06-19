import os
import shutil

# Nastavitev poti do vaših podatkov
base_path = r"C:\Users\jasar\Desktop\FB_roze\Flowers299"  # Uporabite raw string
grass_images_path = r"C:\Users\jasar\Desktop\FB_roze\biomass_data\test\images"  # Uporabite raw string
output_path = r"C:\Users\jasar\Desktop\FB_roze\organized_images"  # Nastavite dejansko ciljno mapo

# Ciljni direktoriji
flowers_dir = os.path.join(output_path, "Flowers")
grass_dir = os.path.join(output_path, "Grass")

# Ustvarjanje direktorijev, če ne obstajajo
os.makedirs(flowers_dir, exist_ok=True)
os.makedirs(grass_dir, exist_ok=True)

# Premikanje slik rož
for folder in os.listdir(base_path):
    folder_path = os.path.join(base_path, folder)
    for image in os.listdir(folder_path):
        src_path = os.path.join(folder_path, image)
        dst_path = os.path.join(flowers_dir, image)
        shutil.copy(src_path, dst_path)

# Kopiranje slik trave
for image in os.listdir(grass_images_path):
    src_path = os.path.join(grass_images_path, image)
    dst_path = os.path.join(grass_dir, image)
    shutil.copy(src_path, dst_path)

print("Slike so bile uspešno organizirane!")
