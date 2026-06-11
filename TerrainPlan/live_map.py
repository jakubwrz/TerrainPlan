import sys
import os
import json
import csv
import time
import urllib.request
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg

# Try to find the correct directory
script_dir = os.path.dirname(os.path.abspath(__file__))
if not os.path.exists(os.path.join(script_dir, 'outputs', 'path_config.json')):
    if os.path.exists(os.path.join(script_dir, 'TerrainPlan', 'outputs', 'path_config.json')):
        script_dir = os.path.join(script_dir, 'TerrainPlan')

config_path = os.path.join(script_dir, 'outputs', 'path_config.json')
map_path = os.path.join(script_dir, 'outputs', 'images', 'analyzed_map.png')
if not os.path.exists(map_path):
    map_path = os.path.join(script_dir, 'map.png')

path_csv = os.path.join(script_dir, 'outputs', 'csv', 'path_coordinates.csv')

print(f"Loading config from: {config_path}")
with open(config_path, 'r') as f:
    config = json.load(f)

inv_matrix = config['inverse_matrix']
def real_to_px(rx, ry):
    px = inv_matrix['A'] * rx + inv_matrix['B'] * ry + inv_matrix['C']
    py = inv_matrix['D'] * rx + inv_matrix['E'] * ry + inv_matrix['F']
    return px, py

# Load Path
path_px_x = []
path_px_y = []
if os.path.exists(path_csv):
    with open(path_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rx = float(row['real_x'])
            ry = float(row['real_y'])
            px, py = real_to_px(rx, ry)
            path_px_x.append(px)
            path_px_y.append(py)

# Setup Plot
fig, ax = plt.subplots(figsize=(10, 8))
img = mpimg.imread(map_path)
ax.imshow(img)

# Plot Path
if path_px_x:
    ax.plot(path_px_x, path_px_y, color='magenta', linewidth=2, label='A* Path')
    ax.scatter([path_px_x[0]], [path_px_y[0]], color='lime', s=100, zorder=5, label='Start')
    ax.scatter([path_px_x[-1]], [path_px_y[-1]], color='cyan', s=100, zorder=5, label='Goal')

# Plot Anchors
for aid, (px, py) in config.get('markers', {}).items():
    if aid.startswith('A'):
        ax.scatter([px], [py], color='red', marker='^', s=150, zorder=4)
        ax.annotate(f" {aid}", (px, py), color='white', weight='bold', 
                    bbox=dict(facecolor='red', alpha=0.5, edgecolor='none', pad=1))

# Rover scatter object
rover_scatter = ax.scatter([], [], color='yellow', edgecolor='black', s=200, zorder=10, label='Rover (Live)')
ax.legend()
ax.set_title("Live UWB Rover Tracking")
ax.axis('off')

def update(frame):
    try:
        # Fetch live data from uwb_server.py
        req = urllib.request.Request("http://localhost:8080/data")
        with urllib.request.urlopen(req, timeout=1.0) as response:
            data = json.loads(response.read().decode())
            
        pos = data.get('position', {})
        if pos.get('success'):
            rx = pos['x']
            ry = pos['y']
            px, py = real_to_px(rx, ry)
            rover_scatter.set_offsets([[px, py]])
            ax.set_title(f"Live Rover | X:{rx:.2f}m Y:{ry:.2f}m | Anchors: {pos.get('num_anchors')}")
        else:
            ax.set_title(f"Live Rover | Searching for signal... ({data.get('format')})")
    except Exception as e:
        ax.set_title("Live Rover | Waiting for uwb_server.py...")

    return rover_scatter,

ani = animation.FuncAnimation(fig, update, interval=200, blit=False)
plt.tight_layout()
plt.show()
