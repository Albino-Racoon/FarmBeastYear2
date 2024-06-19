import numpy as np
import matplotlib.pyplot as plt

# Function to generate the PGM and YAML files
def generate_pgm_and_yaml(pgm_filename='map.pgm', yaml_filename='map.yaml'):
    width_meters = 8.0
    height_meters = 8.0
    resolution = 0.05

    width_pixels = int(width_meters / resolution)
    height_pixels = int(height_meters / resolution)

    map_data = np.full((height_pixels, width_pixels), 255, dtype=np.uint8)

    with open(pgm_filename, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{width_pixels} {height_pixels}\n'.encode())
        f.write(b'255\n')
        map_data.tofile(f)

    origin_x = 0.0
    origin_y = -height_meters / 2

    yaml_content = f"""image: {pgm_filename}
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

    with open(yaml_filename, 'w') as f:
        f.write(yaml_content)

    print(f'Generated {pgm_filename} and {yaml_filename}')

# Function to visualize the map with a point moved by a vector
def visualize_movement(initial_point, vector, resolution=0.05, width_meters=8.0, height_meters=8.0):
    width_pixels = int(width_meters / resolution)
    height_pixels = int(height_meters / resolution)

    fig, ax = plt.subplots()
    ax.set_xlim(0, width_pixels)
    ax.set_ylim(0, height_pixels)

    ax.set_xticks(np.arange(0, width_pixels, width_pixels / 8))
    ax.set_yticks(np.arange(0, height_pixels, height_pixels / 8))
    ax.set_xticklabels(np.round(np.arange(0, width_meters, width_meters / 8) - width_meters / 2, 2))
    ax.set_yticklabels(np.round(np.arange(0, height_meters, height_meters / 8) - height_meters / 2, 2))
    ax.grid(True)

    initial_point_pixels = (initial_point[0] / resolution + width_pixels / 2,
                            initial_point[1] / resolution + height_pixels / 2)
    final_point = (initial_point[0] + vector[0], initial_point[1] + vector[1])
    final_point_pixels = (final_point[0] / resolution + width_pixels / 2,
                          final_point[1] / resolution + height_pixels / 2)

    ax.plot(initial_point_pixels[0], initial_point_pixels[1], 'ro')
    ax.plot(final_point_pixels[0], final_point_pixels[1], 'bo')
    ax.plot([initial_point_pixels[0], final_point_pixels[0]],
            [initial_point_pixels[1], final_point_pixels[1]], 'r-')

    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Movement on the Map')
    plt.show()

# Generate the map files
generate_pgm_and_yaml()

# Define the initial point and vector
initial_point = (0, 0)  # Origin (center of the left side)
#vector = (3, 3)  # Move 3 meters right and 3 meters up

# Visualize the movement
for i in range(10):
    input_x=int(input("x: "))
    input_y=int(input("y: "))
    vector=(input_x,input_y)
    visualize_movement(initial_point, vector)
