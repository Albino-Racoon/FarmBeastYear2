import numpy as np

def generate_ppm_and_yaml(ppm_filename='map.ppm', yaml_filename='map.yaml'):
    # Map parameters
    width_meters = 8.0
    height_meters = 8.0
    resolution = 0.05  # meters per pixel

    # Calculate the size in pixels
    width_pixels = int(width_meters / resolution)
    height_pixels = int(height_meters / resolution)

    # Create a blank (free space) map with white background
    map_data = np.full((height_pixels, width_pixels, 3), 255, dtype=np.uint8)

    # Add borders to the map
    border_thickness = 5  # thickness of the border in pixels
    map_data[:border_thickness, :, 0] = 255  # top border red
    map_data[:border_thickness, :, 1:] = 0  # top border green and blue
    map_data[-border_thickness:, :, 0] = 255  # bottom border red
    map_data[-border_thickness:, :, 1:] = 0  # bottom border green and blue
    map_data[:, :border_thickness, 0] = 255  # left border red
    map_data[:, :border_thickness, 1:] = 0  # left border green and blue
    map_data[:, -border_thickness:, 0] = 255  # right border red
    map_data[:, -border_thickness:, 1:] = 0  # right border green and blue

    # Save the map as a PPM file
    with open(ppm_filename, 'wb') as f:
        f.write(b'P6\n')
        f.write(f'{width_pixels} {height_pixels}\n'.encode())
        f.write(b'255\n')
        map_data.tofile(f)

    # Calculate the origin of the map (center of the left side)
    origin_x = 0.0
    origin_y = -height_meters / 2

    # Create the YAML file content
    yaml_content = f"""image: {ppm_filename}
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

    # Save the YAML file
    with open(yaml_filename, 'w') as f:
        f.write(yaml_content)

    print(f'Generated {ppm_filename} and {yaml_filename}')

# Call the function to generate the files
generate_ppm_and_yaml()
