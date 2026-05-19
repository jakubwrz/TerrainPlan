import csv
import math
import requests
from io import BytesIO
from PIL import Image, ImageDraw
import numpy as np
import os

# Coordinates for a sample area in EPSG:2180 (Poland CS92)
# You can change these to your specific area of interest.
MIN_X = 635000.0 
MAX_X = 635100.0
MIN_Y = 485000.0
MAX_Y = 485100.0

GRID_CELL_SIZE = 1.0 # 1 meter per cell


def classify_terrain(r, g, b):
    """
    Simple heuristic to classify terrain based on RGB color from satellite image.
    In a real scenario, use more advanced image processing or a dedicated land-cover map.
    """
    color_map = {
        'GRASS': (70, 100, 50),
        'CONCRETE': (150, 150, 150),
        'ASPHALT': (100, 100, 100),
        'DIRT': (120, 100, 70),
        'WATER': (50, 100, 150),
        'BUILDING/ROOF': (160, 90, 80)
    }
    
    best_terrain = 'UNKNOWN'
    min_dist = float('inf')
    
    for terrain, (tr, tg, tb) in color_map.items():
        dist = math.sqrt((int(r) - tr)**2 + (int(g) - tg)**2 + (int(b) - tb)**2)
        if dist < min_dist:
            min_dist = dist
            best_terrain = terrain
            
    return best_terrain

def get_height_from_map(x, y, minx, miny, maxx, maxy, heightmap_array):
    """
    Extracts height from a loaded heightmap array based on coordinates.
    """
    if heightmap_array is None:
        # Fallback to dummy height if no heightmap is provided
        return 100.0 + math.sin(x / 10.0) * 2.0 + math.cos(y / 10.0) * 2.0
        
    h, w = heightmap_array.shape[:2]
    
    # Calculate proportional position
    px_x = int(((x - minx) / (maxx - minx)) * w)
    px_y = int((1.0 - ((y - miny) / (maxy - miny))) * h) # Y is inverted
    
    # Bound check
    px_x = max(0, min(px_x, w - 1))
    px_y = max(0, min(px_y, h - 1))
    
    # Return the height value (assuming it's a float/int array)
    val = heightmap_array[px_y, px_x]
    
    # If the TIFF was RGB by mistake, take the first channel
    if isinstance(val, (list, tuple, np.ndarray)):
        return float(val[0])
    return float(val)

def generate_terrain_grid(image, minx, miny, maxx, maxy, cell_size, heightmap_array=None):
    """
    Splits the map into a grid and analyzes each cell.
    """
    width, height = image.size
    img_data = np.array(image)
    
    num_cells_x = int((maxx - minx) / cell_size)
    num_cells_y = int((maxy - miny) / cell_size)
    
    print(f"Generating grid: {num_cells_x} x {num_cells_y} cells...")
    
    grid_data = []
    
    for cx in range(num_cells_x):
        for cy in range(num_cells_y):
            # Calculate real-world coordinates for the center of the cell
            real_x = minx + (cx + 0.5) * cell_size
            real_y = miny + (cy + 0.5) * cell_size
            
            # Map cell back to image pixels
            px_x = int((cx / num_cells_x) * width)
            px_y = int((1.0 - (cy / num_cells_y)) * height) # Y axis is typically inverted in images
            
            # Ensure within bounds
            px_x = max(0, min(px_x, width - 1))
            px_y = max(0, min(px_y, height - 1))
            
            # Get RGB from image
            if len(img_data.shape) == 3 and img_data.shape[2] >= 3:
                r, g, b = img_data[px_y, px_x][:3]
            else:
                # Grayscale or other format, fallback
                r = g = b = img_data[px_y, px_x] if len(img_data.shape) == 2 else 0
            
            terrain_type = classify_terrain(r, g, b)
            h = get_height_from_map(real_x, real_y, minx, miny, maxx, maxy, heightmap_array)
            
            grid_data.append({
                'grid_x': cx,
                'grid_y': cy,
                'real_x': round(real_x, 2),
                'real_y': round(real_y, 2),
                'height': round(h, 2),
                'terrain_type': terrain_type,
                'r': int(r),
                'g': int(g),
                'b': int(b)
            })
            
    return grid_data

def generate_visualization(image, grid_data, out_filename):
    """
    Creates an overlay image to visualize the generated grid.
    """
    print(f"Generating visualization overlay to {out_filename}...")
    
    # Create a transparent overlay layer
    overlay = Image.new('RGBA', image.size, (255, 255, 255, 0))
    draw = ImageDraw.Draw(overlay)
    
    # Define colors for terrain types (R, G, B, Alpha)
    vis_colors = {
        'GRASS': (0, 255, 0, 80),        # Semi-transparent green
        'CONCRETE': (200, 200, 200, 80), # Light gray
        'ASPHALT': (50, 50, 50, 80),     # Dark gray
        'DIRT': (139, 69, 19, 80),       # Brown
        'WATER': (0, 0, 255, 80),        # Blue
        'BUILDING/ROOF': (255, 0, 0, 80) # Red
    }
    
    width, height = image.size
    
    if len(grid_data) > 0:
        # Determine how many cells we have to figure out pixel sizes
        max_grid_x = max(d['grid_x'] for d in grid_data)
        max_grid_y = max(d['grid_y'] for d in grid_data)
        
        num_cells_x = max_grid_x + 1
        num_cells_y = max_grid_y + 1
        
        for d in grid_data:
            cx = d['grid_x']
            cy = d['grid_y']
            t_type = d['terrain_type']
            
            # Map grid index back to pixel bounding box
            px1 = int((cx / num_cells_x) * width)
            py1 = int((1.0 - ((cy + 1) / num_cells_y)) * height) # Inverted Y
            px2 = int(((cx + 1) / num_cells_x) * width)
            py2 = int((1.0 - (cy / num_cells_y)) * height)
            
            # Get color, default to magenta if unknown
            fill_color = vis_colors.get(t_type, (255, 0, 255, 80))
            
            # Draw the semi-transparent rectangle
            draw.rectangle([px1, py1, px2, py2], fill=fill_color)
            
            # Optionally draw a tiny dot in the center of the cell for the exact sample point
            center_x = (px1 + px2) // 2
            center_y = (py1 + py2) // 2
            draw.rectangle([center_x, center_y, center_x+1, center_y+1], fill=(0, 0, 0, 255))
            
    # Composite the original image with the overlay
    if image.mode != 'RGBA':
        image = image.convert('RGBA')
        
    final_img = Image.alpha_composite(image, overlay)
    final_img.convert('RGB').save(out_filename)
    print("Visualization saved successfully!")

def generate_height_visualization(image, grid_data, out_filename):
    """
    Creates an overlay image to visualize the generated heightmap grid.
    Uses a blue (low) to red (high) colormap.
    """
    print(f"Generating height visualization overlay to {out_filename}...")
    
    if len(grid_data) == 0:
        return
        
    overlay = Image.new('RGBA', image.size, (255, 255, 255, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = image.size
    
    # Find min and max heights for normalization
    heights = [d['height'] for d in grid_data]
    min_h = min(heights)
    max_h = max(heights)
    
    max_grid_x = max(d['grid_x'] for d in grid_data)
    max_grid_y = max(d['grid_y'] for d in grid_data)
    num_cells_x = max_grid_x + 1
    num_cells_y = max_grid_y + 1
    
    for d in grid_data:
        cx = d['grid_x']
        cy = d['grid_y']
        h_val = d['height']
        
        px1 = int((cx / num_cells_x) * width)
        py1 = int((1.0 - ((cy + 1) / num_cells_y)) * height)
        px2 = int(((cx + 1) / num_cells_x) * width)
        py2 = int((1.0 - (cy / num_cells_y)) * height)
        
        # Normalize height
        if max_h > min_h:
            norm = (h_val - min_h) / (max_h - min_h)
        else:
            norm = 0.5
            
        # Simple blue to red gradient
        r = int(norm * 255)
        g = int((1.0 - abs(norm - 0.5) * 2.0) * 255)
        b = int((1.0 - norm) * 255)
        
        draw.rectangle([px1, py1, px2, py2], fill=(r, g, b, 120))
        
    if image.mode != 'RGBA':
        image = image.convert('RGBA')
        
    final_img = Image.alpha_composite(image, overlay)
    final_img.convert('RGB').save(out_filename)
    print("Height visualization saved successfully!")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    print("======================================================")
    print("TerrainPlan - Analyzing satellite image for UWB Path Planning")
    print("======================================================")
    
    # Calculate required image dimensions (let's say 1 pixel per 10cm for decent resolution)
    width = int((MAX_X - MIN_X) * 10)
    height = int((MAX_Y - MIN_Y) * 10)
    

    print("\nFalling back to local 'map' files if they exist...")
    if os.path.exists(os.path.join(script_dir, 'map.png')):
        image = Image.open(os.path.join(script_dir, 'map.png'))
        print("Loaded local 'map.png'.")
    elif os.path.exists(os.path.join(script_dir, 'map.tif')):
        image = Image.open(os.path.join(script_dir, 'map.tif'))
        print("Loaded local 'map.tif'.")
    elif os.path.exists(os.path.join(script_dir, 'map.tiff')):
        image = Image.open(os.path.join(script_dir, 'map.tiff'))
        print("Loaded local 'map.tiff'.")
    else:
        print("Could not fetch image from Geoportal and no local 'map' file was found.")
        print("Generating a dummy green image for demonstration...")
        image = Image.new('RGB', (width, height), color = (75, 110, 50))
    
    if image.mode != 'RGB':
        image = image.convert('RGB')
        
    # Save a copy of the image being analyzed so the user can see it
    out_img_path = os.path.join(script_dir, 'analyzed_map.png')
    image.save(out_img_path)
    print(f"Saved the image being analyzed to '{out_img_path}'.")
        
    # 2. Fetch Heightmap
    heightmap_array = None
    print("\nLooking for local 'heightmap.tif' or 'heightmap.tiff' for actual elevation data...")
    if os.path.exists(os.path.join(script_dir, 'heightmap.tif')):
        try:
            h_img = Image.open(os.path.join(script_dir, 'heightmap.tif'))
            heightmap_array = np.array(h_img)
            print("Loaded real heightmap data from 'heightmap.tif'!")
        except Exception as e:
            print(f"Failed to load 'heightmap.tif': {e}")
    elif os.path.exists(os.path.join(script_dir, 'heightmap.tiff')):
        try:
            h_img = Image.open(os.path.join(script_dir, 'heightmap.tiff'))
            heightmap_array = np.array(h_img)
            print("Loaded real heightmap data from 'heightmap.tiff'!")
        except Exception as e:
            print(f"Failed to load 'heightmap.tiff': {e}")
    else:
        print("No heightmap found. Falling back to generated dummy heights.")
        
    # 3. Analyze and Generate Grid
    grid = generate_terrain_grid(image, MIN_X, MIN_Y, MAX_X, MAX_Y, GRID_CELL_SIZE, heightmap_array)
    
    # 4. Export to CSV
    csv_filename = os.path.join(script_dir, "terrain_grid.csv")
    print(f"\nExporting data to {csv_filename}...")
    with open(csv_filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['grid_x', 'grid_y', 'real_x', 'real_y', 'height', 'terrain_type', 'r', 'g', 'b'])
        writer.writeheader()
        writer.writerows(grid)
        
    print(f"Success! Grid data exported to {csv_filename}")
    print(f"Total grid points generated: {len(grid)}")
    
    # 5. Generate Visualizations
    vis_filename = os.path.join(script_dir, "visualization_overlay.png")
    generate_visualization(image, grid, vis_filename)
    
    height_vis_filename = os.path.join(script_dir, "visualization_heights.png")
    generate_height_visualization(image, grid, height_vis_filename)
    
    print("You can use this CSV for your UWB Path Planning algorithm.")

if __name__ == "__main__":
    main()
