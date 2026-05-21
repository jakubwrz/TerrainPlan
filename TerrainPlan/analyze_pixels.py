"""Sample pixel colors from test images to determine training coordinates."""
from PIL import Image
import numpy as np
import json, os

def analyze_image(path, name):
    img = Image.open(path).convert('RGB')
    data = np.array(img)
    h, w = data.shape[:2]
    print(f"\n=== {name} === ({w}x{h})")
    
    # Sample a grid of points and print their colors
    points = []
    for py in range(0, h, h//8):
        for px in range(0, w, w//8):
            py_c = min(py, h-1)
            px_c = min(px, w-1)
            # Average 5x5 patch
            y0, y1 = max(0,py_c-2), min(h,py_c+3)
            x0, x1 = max(0,px_c-2), min(w,px_c+3)
            patch = data[y0:y1, x0:x1]
            avg = patch.mean(axis=(0,1))
            r, g, b = int(avg[0]), int(avg[1]), int(avg[2])
            
            # Compute green ratio and excess green
            total = r + g + b + 1
            green_ratio = g / total
            excess_green = 2*g - r - b
            
            points.append({
                'x': px_c, 'y': py_c, 
                'r': r, 'g': g, 'b': b,
                'gr': round(green_ratio, 3),
                'exg': excess_green
            })
    
    # Group by likely terrain type based on visual inspection
    for p in points:
        r, g, b = p['r'], p['g'], p['b']
        brightness = (r + g + b) / 3
        if brightness > 160:
            guess = "CONCRETE?"
        elif brightness > 100 and p['gr'] < 0.38:
            guess = "DIRT?"
        elif p['exg'] > 20 and brightness > 60:
            guess = "GRASS?"
        elif p['exg'] > 0 and brightness <= 60:
            guess = "TREE?"
        elif brightness < 40:
            guess = "SHADOW/WATER?"
        else:
            guess = "???"
        
        print(f"  ({p['x']:3d},{p['y']:3d}) RGB=({r:3d},{g:3d},{b:3d}) bright={brightness:.0f} gr={p['gr']:.3f} exg={p['exg']:+4d}  -> {guess}")

script_dir = os.path.dirname(os.path.abspath(__file__))
analyze_image(os.path.join(script_dir, 'map.tiff'), 'Current (map.tiff)')
analyze_image(os.path.join(script_dir, 'PARK_GOOD_1/map.tiff'), 'PARK_GOOD_1')
analyze_image(os.path.join(script_dir, 'RIVER_DECENT_1/map.tiff'), 'RIVER_DECENT_1')
analyze_image(os.path.join(script_dir, 'WAL_GOOD_1/map.tiff'), 'WAL_GOOD_1')
