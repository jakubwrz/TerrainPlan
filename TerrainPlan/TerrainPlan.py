import csv
import math
import json
import colorsys
from PIL import Image, ImageDraw
import numpy as np
import os
from collections import Counter

# Coordinates for a sample area in EPSG:2180 (Poland CS92)
# You can change these to your specific area of interest.
MIN_X = 635000.0 
MAX_X = 635100.0
MIN_Y = 485000.0
MAX_Y = 485100.0

GRID_CELL_SIZE = 0.2 # 0.2 meters (20 cm) per cell (increased resolution from 1.0)

# Number of pixels to average around each cell center (NxN neighborhood).
# Larger values reduce noise but blur boundaries between terrain types.
PIXEL_AVERAGE_RADIUS = 2  # 5x5 pixel neighborhood (2 pixels in each direction)

# Enable spatial majority-vote smoothing after initial classification.
ENABLE_SPATIAL_SMOOTHING = True

# Name of the training samples file (looked for in the same directory as the map image).
TRAINING_SAMPLES_FILENAME = "training_samples.json"


def get_geotiff_bounds(image):
    """
    Attempt to extract coordinate bounds from GeoTIFF tags.
    Returns (min_x, min_y, max_x, max_y) or None if tags are missing or invalid.
    """
    if not hasattr(image, 'tag'):
        return None
    try:
        # Tag 33922 is ModelTiepointTag: (dx, dy, dz, X0, Y0, Z0)
        # Tag 33550 is ModelPixelScaleTag: (scale_x, scale_y, scale_z)
        tiepoint = image.tag.get(33922)
        scale = image.tag.get(33550)
        if tiepoint and scale and len(tiepoint) >= 6 and len(scale) >= 2:
            x0, y0 = tiepoint[3], tiepoint[4]
            scale_x, scale_y = scale[0], scale[1]
            w, h = image.size
            min_x = x0
            max_x = x0 + w * scale_x
            min_y = y0 - h * scale_y
            max_y = y0
            return min_x, min_y, max_x, max_y
    except Exception as e:
        print(f"  Warning: Error parsing GeoTIFF tags: {e}")
    return None


# =============================================================================
# FEATURE EXTRACTION
# =============================================================================

def compute_feature_vector(r, g, b, local_std, local_gradient):
    """
    Compute a multi-dimensional feature vector for a pixel/cell.
    
    Features capture both color properties and local texture, making the
    classifier robust to shadows (HSV separates brightness from color),
    vegetation (green indices), and surface texture (std dev, gradient).
    
    Args:
        r, g, b: Average RGB values (0-255)
        local_std: Standard deviation of brightness in the local neighborhood
        local_gradient: Mean gradient magnitude in the local neighborhood
        
    Returns:
        numpy array of shape (10,) with normalized features
    """
    # Normalize RGB to 0-1
    r_n = r / 255.0
    g_n = g / 255.0
    b_n = b / 255.0
    
    # HSV (Hue, Saturation, Value) - separates color from brightness
    h, s, v = colorsys.rgb_to_hsv(r_n, g_n, b_n)
    
    # Vegetation indices (standard remote sensing features)
    total = r + g + b + 1e-6  # avoid division by zero
    green_ratio = g / total    # how "green" relative to total brightness
    excess_green = (2.0 * g - r - b) / 255.0  # normalized Excess Green Index
    
    # Brightness
    brightness = (r + g + b) / (3.0 * 255.0)
    
    return np.array([
        h,               # 0: Hue (0-1, wraps at 1)
        s,               # 1: Saturation (0-1)
        v,               # 2: Value/brightness (0-1)
        green_ratio,     # 3: Green ratio G/(R+G+B)
        excess_green,    # 4: Excess Green Index (2G-R-B)/255
        local_std / 255.0,     # 5: Local texture (stddev of brightness)
        local_gradient / 255.0 # 6: Local edge strength
    ], dtype=np.float32)


def compute_local_texture(img_data, cx, cy, radius=3):
    """
    Compute local texture features for a pixel neighborhood.
    
    Args:
        img_data: numpy array (H, W, 3)
        cx, cy: center pixel coordinates
        radius: neighborhood radius
        
    Returns:
        (local_std, local_gradient): texture features
    """
    h, w = img_data.shape[:2]
    
    y0 = max(0, cy - radius)
    y1 = min(h, cy + radius + 1)
    x0 = max(0, cx - radius)
    x1 = min(w, cx + radius + 1)
    
    patch = img_data[y0:y1, x0:x1].astype(np.float32)
    
    # Convert to grayscale for texture analysis
    if patch.ndim == 3:
        gray_patch = patch[:, :, 0] * 0.299 + patch[:, :, 1] * 0.587 + patch[:, :, 2] * 0.114
    else:
        gray_patch = patch
    
    # Standard deviation of brightness (smooth surfaces have low std)
    local_std = float(np.std(gray_patch))
    
    # Gradient magnitude (edges between terrain types)
    if gray_patch.shape[0] >= 3 and gray_patch.shape[1] >= 3:
        gy = np.diff(gray_patch, axis=0)
        gx = np.diff(gray_patch, axis=1)
        # Average gradient magnitude
        grad_mag = np.sqrt(gy[:gx.shape[0], :gx.shape[1]]**2 + gx[:gy.shape[0], :gy.shape[1]]**2)
        local_gradient = float(np.mean(grad_mag))
    else:
        local_gradient = 0.0
    
    return local_std, local_gradient


def get_averaged_rgb(img_data, center_x, center_y, radius, img_width, img_height):
    """
    Average RGB values over a (2*radius+1) x (2*radius+1) pixel neighborhood.
    """
    x_min = max(0, center_x - radius)
    x_max = min(img_width - 1, center_x + radius)
    y_min = max(0, center_y - radius)
    y_max = min(img_height - 1, center_y + radius)
    
    patch = img_data[y_min:y_max+1, x_min:x_max+1, :3]
    avg = patch.mean(axis=(0, 1))
    
    return float(avg[0]), float(avg[1]), float(avg[2])


# =============================================================================
# ML CLASSIFIER (Random Forest)
# =============================================================================

def load_training_samples(json_path, img_data):
    """
    Load training samples from a JSON file and extract feature vectors
    from the labeled pixel regions.
    
    Args:
        json_path: path to training_samples.json
        img_data: numpy array of the image (H, W, 3)
        
    Returns:
        (X, y, label_names) where X is feature matrix, y is label indices,
        label_names maps index -> terrain type string.
        Returns (None, None, None) if file doesn't exist or is invalid.
    """
    if not os.path.exists(json_path):
        return None, None, None
    
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except (json.JSONDecodeError, IOError) as e:
        print(f"  Warning: Could not read training samples file: {e}")
        return None, None, None
    
    h, w = img_data.shape[:2]
    
    features_list = []
    labels_list = []
    label_names = {}
    label_idx = 0
    
    for sample_group in data.get('samples', []):
        terrain_type = sample_group['terrain']
        if terrain_type == 'TREE/CANOPY':
            terrain_type = 'GRASS'
        
        if terrain_type not in label_names.values():
            label_names[label_idx] = terrain_type
            current_idx = label_idx
            label_idx += 1
        else:
            # Find existing index
            current_idx = [k for k, v in label_names.items() if v == terrain_type][0]
        
        for region in sample_group.get('regions', []):
            rx, ry = region['x'], region['y']
            rw, rh = region.get('w', 10), region.get('h', 10)
            
            # Clamp region to image bounds
            x0 = max(0, min(rx, w - 1))
            y0 = max(0, min(ry, h - 1))
            x1 = max(0, min(rx + rw, w))
            y1 = max(0, min(ry + rh, h))
            
            if x1 <= x0 or y1 <= y0:
                continue
            
            # Sample pixels from the region (every pixel, or subsampled if large)
            step = max(1, min(rw, rh) // 10)  # subsample very large regions
            for py in range(y0, y1, step):
                for px in range(x0, x1, step):
                    r, g, b = get_averaged_rgb(img_data, px, py, 1, w, h)
                    local_std, local_gradient = compute_local_texture(img_data, px, py, radius=3)
                    fv = compute_feature_vector(r, g, b, local_std, local_gradient)
                    
                    features_list.append(fv)
                    labels_list.append(current_idx)
    
    if len(features_list) == 0:
        print("  Warning: No valid training samples found in JSON file.")
        return None, None, None
    
    X = np.array(features_list)
    y = np.array(labels_list)
    
    return X, y, label_names


def load_all_training_samples(start_dir):
    """
    Search recursively starting from start_dir for all subdirectories containing
    training_samples.json and a corresponding map image. Load and combine all
    samples into a single training set.
    """
    global_features = []
    global_labels = []
    
    # We want a unified mapping of label string -> integer
    terrain_types = [
        'GRASS',
        'CONCRETE',
        'ASPHALT',
        'DIRT',
        'WATER',
        'BUILDING/ROOF',
    ]
    label_to_idx = {name: idx for idx, name in enumerate(terrain_types)}
    idx_to_label = {idx: name for idx, name in enumerate(terrain_types)}
    
    print("Searching for training samples across the project...")
    
    for root, dirs, files in os.walk(start_dir):
        if TRAINING_SAMPLES_FILENAME in files:
            json_path = os.path.join(root, TRAINING_SAMPLES_FILENAME)
            # Find map image in the same directory
            image_path = None
            for ext in ['map.png', 'map.tif', 'map.tiff']:
                candidate = os.path.join(root, ext)
                if os.path.exists(candidate):
                    image_path = candidate
                    break
            
            if image_path:
                print(f"  Loading training samples from: {root}")
                try:
                    img = Image.open(image_path).convert('RGB')
                    img_data = np.array(img)
                    
                    with open(json_path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    h, w = img_data.shape[:2]
                    count = 0
                    for sample_group in data.get('samples', []):
                        terrain_type = sample_group['terrain']
                        if terrain_type == 'TREE/CANOPY':
                            terrain_type = 'GRASS'
                        
                        if terrain_type not in label_to_idx:
                            new_idx = len(label_to_idx)
                            label_to_idx[terrain_type] = new_idx
                            idx_to_label[new_idx] = terrain_type
                        
                        current_idx = label_to_idx[terrain_type]
                        
                        for region in sample_group.get('regions', []):
                            rx, ry = region['x'], region['y']
                            rw, rh = region.get('w', 10), region.get('h', 10)
                            
                            x0 = max(0, min(rx, w - 1))
                            y0 = max(0, min(ry, h - 1))
                            x1 = max(0, min(rx + rw, w))
                            y1 = max(0, min(ry + rh, h))
                            
                            if x1 <= x0 or y1 <= y0:
                                continue
                            
                            step = max(1, min(rw, rh) // 10)
                            for py in range(y0, y1, step):
                                for px in range(x0, x1, step):
                                    r, g, b = get_averaged_rgb(img_data, px, py, 1, w, h)
                                    local_std, local_gradient = compute_local_texture(img_data, px, py, radius=3)
                                    fv = compute_feature_vector(r, g, b, local_std, local_gradient)
                                    
                                    global_features.append(fv)
                                    global_labels.append(current_idx)
                                    count += 1
                    print(f"    Loaded {count} samples from {os.path.basename(image_path)}")
                except Exception as e:
                    print(f"    Error loading samples from {root}: {e}")
                    
    if len(global_features) == 0:
        return None, None, None
        
    X = np.array(global_features)
    y = np.array(global_labels)
    
    unique_indices = np.unique(y)
    filtered_idx_to_label = {int(idx): idx_to_label[idx] for idx in unique_indices}
    
    return X, y, filtered_idx_to_label


def train_classifier(X, y, label_names):
    """
    Train a Random Forest classifier on the labeled training data.
    
    Random Forest is ideal here because:
    - Handles non-linear decision boundaries (shadow vs. terrain)
    - Works with small training sets (dozens of samples per class)
    - Fast to train and predict
    - No GPU needed
    """
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.model_selection import cross_val_score
    
    print(f"  Training Random Forest classifier...")
    print(f"  Training samples: {len(y)} pixels across {len(label_names)} terrain types")
    
    for idx, name in label_names.items():
        count = np.sum(y == idx)
        print(f"    {name}: {count} samples")
    
    # Train the model
    clf = RandomForestClassifier(
        n_estimators=100,      # 100 decision trees
        max_depth=6,           # prevent overfitting (regularized from 12)
        min_samples_leaf=5,    # minimum 5 samples per leaf (regularized from 3)
        random_state=42,       # reproducible results
        n_jobs=-1              # use all CPU cores
    )
    clf.fit(X, y)
    
    # Cross-validation score (if enough samples)
    if len(y) >= 20:
        n_splits = min(5, min(Counter(y).values()))
        if n_splits >= 2:
            scores = cross_val_score(clf, X, y, cv=n_splits)
            print(f"  Cross-validation accuracy: {scores.mean():.1%} (+/- {scores.std()*2:.1%})")
    
    # Feature importance
    feature_names = ['Hue', 'Saturation', 'Value', 'GreenRatio', 'ExcessGreen', 'LocalStd', 'Gradient']
    importances = clf.feature_importances_
    top_features = sorted(zip(feature_names, importances), key=lambda x: -x[1])
    print(f"  Top features: {', '.join(f'{name}={imp:.2f}' for name, imp in top_features[:5])}")
    
    return clf


# =============================================================================
# HSV FALLBACK CLASSIFIER (used when no training samples exist)
# =============================================================================

def classify_terrain_hsv_fallback(r, g, b):
    """
    Fallback HSV-based classifier when no training samples are available.
    Less accurate than the ML classifier but requires no labeled data.
    """
    r_norm = int(r) / 255.0
    g_norm = int(g) / 255.0
    b_norm = int(b) / 255.0
    
    h, s, v = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
    h_deg = h * 360.0
    
    if 60 <= h_deg <= 220 and v < 0.35 and s > 0.10:
        return 'GRASS'
    if v < 0.12:
        return 'GRASS'
    if 180 <= h_deg <= 260 and s > 0.25 and v > 0.20:
        return 'WATER'
    if 35 <= h_deg <= 165 and s > 0.12:
        return 'GRASS'
    if (h_deg <= 25 or h_deg >= 340) and s > 0.20 and 0.25 <= v <= 0.85:
        return 'BUILDING/ROOF'
    if 10 <= h_deg <= 45 and 0.10 <= s <= 0.65 and 0.20 <= v <= 0.80:
        return 'DIRT'
    if s < 0.18 and v > 0.45:
        return 'CONCRETE'
    if s < 0.18 and 0.12 <= v <= 0.45:
        return 'ASPHALT'
    
    return 'UNKNOWN'


# =============================================================================
# SPATIAL SMOOTHING
# =============================================================================

def apply_spatial_smoothing(grid_data, num_cells_x, num_cells_y):
    """
    Majority-vote spatial smoothing: each cell's terrain type is replaced by
    the most common terrain type in its 3x3 neighborhood.
    """
    print("Applying spatial majority-vote smoothing...")
    
    terrain_grid = {}
    for d in grid_data:
        terrain_grid[(d['grid_x'], d['grid_y'])] = d['terrain_type']
    
    smoothed_types = {}
    for d in grid_data:
        cx, cy = d['grid_x'], d['grid_y']
        neighbor_types = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in terrain_grid:
                    neighbor_types.append(terrain_grid[(nx, ny)])
        
        counter = Counter(neighbor_types)
        smoothed_types[(cx, cy)] = counter.most_common(1)[0][0]
    
    changed = 0
    for d in grid_data:
        new_type = smoothed_types[(d['grid_x'], d['grid_y'])]
        if new_type != d['terrain_type']:
            changed += 1
            d['terrain_type'] = new_type
    
    print(f"  Smoothing changed {changed} out of {len(grid_data)} cells.")
    return grid_data


# =============================================================================
# HEIGHT MAP
# =============================================================================

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


# =============================================================================
# GRID GENERATION
# =============================================================================

def generate_terrain_grid(image, minx, miny, maxx, maxy, cell_size, 
                          heightmap_array=None, classifier=None, label_names=None):
    """
    Splits the map into a grid and analyzes each cell.
    Uses ML classifier if available, otherwise falls back to HSV rules.
    """
    width, height = image.size
    img_data = np.array(image)
    
    num_cells_x = int((maxx - minx) / cell_size)
    num_cells_y = int((maxy - miny) / cell_size)
    
    mode = "ML Random Forest" if classifier is not None else "HSV fallback rules"
    print(f"Generating grid: {num_cells_x} x {num_cells_y} cells...")
    print(f"  Image size: {width} x {height} pixels")
    print(f"  Classification mode: {mode}")
    
    grid_data = []
    is_color = len(img_data.shape) == 3 and img_data.shape[2] >= 3
    
    # If using ML, create reverse lookup for label names
    idx_to_label = label_names if label_names else {}
    
    # Pre-compute all feature vectors for batch prediction (much faster)
    if classifier is not None:
        all_features = []
        all_rgb = []
        
        for cx in range(num_cells_x):
            for cy in range(num_cells_y):
                px_x = int((cx / num_cells_x) * width)
                px_y = int((1.0 - (cy / num_cells_y)) * height)
                px_x = max(0, min(px_x, width - 1))
                px_y = max(0, min(px_y, height - 1))
                
                if is_color:
                    r, g, b = get_averaged_rgb(img_data, px_x, px_y, PIXEL_AVERAGE_RADIUS, width, height)
                else:
                    val = img_data[px_y, px_x] if len(img_data.shape) == 2 else 0
                    r = g = b = float(val)
                
                local_std, local_gradient = compute_local_texture(img_data, px_x, px_y, radius=3)
                fv = compute_feature_vector(r, g, b, local_std, local_gradient)
                
                all_features.append(fv)
                all_rgb.append((r, g, b))
        
        # Batch prediction (much faster than per-pixel)
        X_all = np.array(all_features)
        predictions = classifier.predict(X_all)
        
        # Build grid data
        idx = 0
        for cx in range(num_cells_x):
            for cy in range(num_cells_y):
                real_x = minx + (cx + 0.5) * cell_size
                real_y = miny + (cy + 0.5) * cell_size
                
                r, g, b = all_rgb[idx]
                terrain_type = idx_to_label.get(predictions[idx], 'UNKNOWN')
                h_val = get_height_from_map(real_x, real_y, minx, miny, maxx, maxy, heightmap_array)
                
                # Get HSV for debug output
                r_n, g_n, b_n = r/255.0, g/255.0, b/255.0
                h_hsv, s_hsv, v_hsv = colorsys.rgb_to_hsv(r_n, g_n, b_n)
                
                grid_data.append({
                    'grid_x': cx,
                    'grid_y': cy,
                    'real_x': round(real_x, 2),
                    'real_y': round(real_y, 2),
                    'height': round(h_val, 2),
                    'terrain_type': terrain_type,
                    'r': int(round(r)),
                    'g': int(round(g)),
                    'b': int(round(b)),
                    'hsv_h': round(h_hsv * 360, 1),
                    'hsv_s': round(s_hsv, 3),
                    'hsv_v': round(v_hsv, 3)
                })
                idx += 1
    else:
        # Fallback: HSV rule-based classification
        for cx in range(num_cells_x):
            for cy in range(num_cells_y):
                real_x = minx + (cx + 0.5) * cell_size
                real_y = miny + (cy + 0.5) * cell_size
                
                px_x = int((cx / num_cells_x) * width)
                px_y = int((1.0 - (cy / num_cells_y)) * height)
                px_x = max(0, min(px_x, width - 1))
                px_y = max(0, min(px_y, height - 1))
                
                if is_color:
                    r, g, b = get_averaged_rgb(img_data, px_x, px_y, PIXEL_AVERAGE_RADIUS, width, height)
                else:
                    val = img_data[px_y, px_x] if len(img_data.shape) == 2 else 0
                    r = g = b = float(val)
                
                terrain_type = classify_terrain_hsv_fallback(r, g, b)
                h_val = get_height_from_map(real_x, real_y, minx, miny, maxx, maxy, heightmap_array)
                
                r_n, g_n, b_n = r/255.0, g/255.0, b/255.0
                h_hsv, s_hsv, v_hsv = colorsys.rgb_to_hsv(r_n, g_n, b_n)
                
                grid_data.append({
                    'grid_x': cx,
                    'grid_y': cy,
                    'real_x': round(real_x, 2),
                    'real_y': round(real_y, 2),
                    'height': round(h_val, 2),
                    'terrain_type': terrain_type,
                    'r': int(round(r)),
                    'g': int(round(g)),
                    'b': int(round(b)),
                    'hsv_h': round(h_hsv * 360, 1),
                    'hsv_s': round(s_hsv, 3),
                    'hsv_v': round(v_hsv, 3)
                })
    
    # Apply spatial majority-vote smoothing if enabled
    if ENABLE_SPATIAL_SMOOTHING:
        grid_data = apply_spatial_smoothing(grid_data, num_cells_x, num_cells_y)
    
    # Print terrain distribution summary
    terrain_counts = Counter(d['terrain_type'] for d in grid_data)
    print("\n  Terrain distribution:")
    for terrain, count in terrain_counts.most_common():
        pct = 100.0 * count / len(grid_data)
        print(f"    {terrain:15s}: {count:5d} cells ({pct:5.1f}%)")
    
    return grid_data


# =============================================================================
# VISUALIZATION
# =============================================================================

def generate_visualization(image, grid_data, out_filename):
    """
    Creates an overlay image to visualize the generated grid.
    """
    print(f"Generating visualization overlay to {out_filename}...")
    
    overlay = Image.new('RGBA', image.size, (255, 255, 255, 0))
    draw = ImageDraw.Draw(overlay)
    
    vis_colors = {
        'GRASS': (0, 255, 0, 80),          # Bright green
        'CONCRETE': (200, 200, 200, 80),   # Light gray
        'ASPHALT': (50, 50, 50, 80),       # Dark gray
        'DIRT': (139, 69, 19, 80),         # Brown
        'WATER': (0, 0, 255, 80),          # Blue
        'BUILDING/ROOF': (255, 0, 0, 80),  # Red
        'TREE/CANOPY': (0, 100, 0, 80),    # Dark green
    }
    
    width, height = image.size
    
    if len(grid_data) > 0:
        max_grid_x = max(d['grid_x'] for d in grid_data)
        max_grid_y = max(d['grid_y'] for d in grid_data)
        num_cells_x = max_grid_x + 1
        num_cells_y = max_grid_y + 1
        
        for d in grid_data:
            cx = d['grid_x']
            cy = d['grid_y']
            t_type = d['terrain_type']
            
            px1 = int((cx / num_cells_x) * width)
            py1 = int((1.0 - ((cy + 1) / num_cells_y)) * height)
            px2 = int(((cx + 1) / num_cells_x) * width)
            py2 = int((1.0 - (cy / num_cells_y)) * height)
            
            fill_color = vis_colors.get(t_type, (255, 0, 255, 80))
            draw.rectangle([px1, py1, px2, py2], fill=fill_color)
            
            center_x = (px1 + px2) // 2
            center_y = (py1 + py2) // 2
            draw.rectangle([center_x, center_y, center_x+1, center_y+1], fill=(0, 0, 0, 255))
            
    if image.mode != 'RGBA':
        image = image.convert('RGBA')
        
    final_img = Image.alpha_composite(image, overlay)
    final_img.convert('RGB').save(out_filename)
    print("Visualization saved successfully!")


def generate_training_visualization(image, json_path, out_filename):
    """
    Creates a visualization showing which pixel regions are used for training.
    Helps verify that your labeled regions are correct.
    """
    if not os.path.exists(json_path):
        return
    
    try:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except (json.JSONDecodeError, IOError):
        return
    
    print(f"Generating training samples visualization to {out_filename}...")
    
    vis_colors = {
        'GRASS': (0, 255, 0),
        'CONCRETE': (200, 200, 200),
        'ASPHALT': (50, 50, 50),
        'DIRT': (139, 69, 19),
        'WATER': (0, 0, 255),
        'BUILDING/ROOF': (255, 0, 0),
        'TREE/CANOPY': (0, 100, 0),
    }
    
    img_copy = image.copy()
    if img_copy.mode != 'RGB':
        img_copy = img_copy.convert('RGB')
    draw = ImageDraw.Draw(img_copy)
    
    for sample_group in data.get('samples', []):
        terrain_type = sample_group['terrain']
        color = vis_colors.get(terrain_type, (255, 0, 255))
        
        for region in sample_group.get('regions', []):
            rx, ry = region['x'], region['y']
            rw, rh = region.get('w', 10), region.get('h', 10)
            
            # Draw rectangle outline (3px thick)
            for offset in range(3):
                draw.rectangle(
                    [rx - offset, ry - offset, rx + rw + offset, ry + rh + offset],
                    outline=color
                )
    
    img_copy.save(out_filename)
    print("Training visualization saved successfully!")


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


# =============================================================================
# MAIN
# =============================================================================

def main():
    import sys
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Parse target directory argument
    target_dir = script_dir
    if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
        arg_dir = sys.argv[1]
        if os.path.isabs(arg_dir):
            target_dir = arg_dir
        else:
            target_dir = os.path.abspath(os.path.join(script_dir, arg_dir))
            
    print("======================================================")
    print("TerrainPlan - Analyzing satellite image for UWB Path Planning")
    print(f"Target directory: {target_dir}")
    print("======================================================")
    
    # Locate map image in target directory
    image = None
    for ext in ['map.png', 'map.tif', 'map.tiff']:
        candidate = os.path.join(target_dir, ext)
        if os.path.exists(candidate):
            try:
                image = Image.open(candidate)
                print(f"Loaded '{ext}' from target directory.")
                break
            except Exception as e:
                print(f"Failed to load '{candidate}': {e}")
                
    if image is None:
        print("Could not find any map image (map.png, map.tif, map.tiff) in target directory.")
        # Fallback to dummy image
        width = int((MAX_X - MIN_X) * 10)
        height = int((MAX_Y - MIN_Y) * 10)
        print("Generating a dummy green image for demonstration...")
        image = Image.new('RGB', (width, height), color = (75, 110, 50))
    
    if image.mode != 'RGB':
        image = image.convert('RGB')
        
    # Save a copy of the image being analyzed so the user can see it
    out_img_path = os.path.join(target_dir, 'analyzed_map.png')
    image.save(out_img_path)
    print(f"Saved the image being analyzed to '{out_img_path}'.")
    
    # Dynamic coordinate bounds extraction
    bounds = get_geotiff_bounds(image)
    if bounds:
        min_x, min_y, max_x, max_y = bounds
        print(f"Detected georeferenced GeoTIFF bounds:")
        print(f"  X: [{min_x:.2f}, {max_x:.2f}] (width: {max_x - min_x:.2f} m)")
        print(f"  Y: [{min_y:.2f}, {max_y:.2f}] (height: {max_y - min_y:.2f} m)")
    else:
        min_x, min_y, max_x, max_y = MIN_X, MIN_Y, MAX_X, MAX_Y
        print("No GeoTIFF bounds found. Falling back to default coordinate bounds:")
        print(f"  X: [{min_x:.2f}, {max_x:.2f}]")
        print(f"  Y: [{min_y:.2f}, {max_y:.2f}]")
        
    # --- ML CLASSIFIER TRAINING ---
    img_data = np.array(image)
    training_json_path = os.path.join(target_dir, TRAINING_SAMPLES_FILENAME)
    
    classifier = None
    label_names = None
    
    print(f"\nLooking for training samples globally starting from: {script_dir}")
    X_train, y_train, label_names = load_all_training_samples(script_dir)
    
    if X_train is not None:
        print(f"  Found {len(y_train)} total training samples globally!")
        classifier = train_classifier(X_train, y_train, label_names)
        
        # Generate training visualization for target directory if JSON exists
        if os.path.exists(training_json_path):
            train_vis_path = os.path.join(target_dir, "visualization_training.png")
            generate_training_visualization(image, training_json_path, train_vis_path)
    else:
        print("  No training samples found globally. Using HSV fallback classifier.")
        
    # 2. Fetch Heightmap
    heightmap_array = None
    print("\nLooking for local 'heightmap.tif' or 'heightmap.tiff' in target directory for actual elevation data...")
    if os.path.exists(os.path.join(target_dir, 'heightmap.tif')):
        try:
            h_img = Image.open(os.path.join(target_dir, 'heightmap.tif'))
            heightmap_array = np.array(h_img)
            print("Loaded real heightmap data from 'heightmap.tif'!")
        except Exception as e:
            print(f"Failed to load 'heightmap.tif': {e}")
    elif os.path.exists(os.path.join(target_dir, 'heightmap.tiff')):
        try:
            h_img = Image.open(os.path.join(target_dir, 'heightmap.tiff'))
            heightmap_array = np.array(h_img)
            print("Loaded real heightmap data from 'heightmap.tiff'!")
        except Exception as e:
            print(f"Failed to load 'heightmap.tiff': {e}")
    else:
        print("No heightmap found. Falling back to generated dummy heights.")
        
    # 3. Analyze and Generate Grid
    grid = generate_terrain_grid(image, min_x, min_y, max_x, max_y, GRID_CELL_SIZE, 
                                 heightmap_array, classifier, label_names)
    
    # 4. Export to CSV (includes HSV debug columns)
    csv_filename = os.path.join(target_dir, "terrain_grid.csv")
    print(f"\nExporting data to {csv_filename}...")
    with open(csv_filename, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'grid_x', 'grid_y', 'real_x', 'real_y', 'height', 
            'terrain_type', 'r', 'g', 'b', 'hsv_h', 'hsv_s', 'hsv_v'
        ])
        writer.writeheader()
        writer.writerows(grid)
        
    print(f"Success! Grid data exported to {csv_filename}")
    print(f"Total grid points generated: {len(grid)}")
    
    # 5. Generate Visualizations
    vis_filename = os.path.join(target_dir, "visualization_overlay.png")
    generate_visualization(image, grid, vis_filename)
    
    height_vis_filename = os.path.join(target_dir, "visualization_heights.png")
    generate_height_visualization(image, grid, height_vis_filename)
    
    print("You can use this CSV for your UWB Path Planning algorithm.")

if __name__ == "__main__":
    main()
