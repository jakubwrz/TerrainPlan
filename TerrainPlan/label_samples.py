"""
Interactive Training Sample Labeler for TerrainPlan.

Opens the satellite image and lets you click to define training regions
for each terrain type. Saves the results as training_samples.json.

Usage:
    python label_samples.py [path_to_map_image]
    
    If no path given, defaults to 'map.tiff' in the current directory.
    
Controls:
    - Select terrain type with number keys (shown in the console)
    - Click and drag to draw a rectangle region
    - Press 'S' to save and quit
    - Press 'Q' to quit without saving
    - Press 'U' to undo the last region
    - Press 'C' to clear all regions for the current terrain type
"""

import sys
import os
import json
from PIL import Image

# Try to use tkinter for a GUI approach
try:
    import tkinter as tk
    HAS_TK = True
except ImportError:
    HAS_TK = False


TERRAIN_TYPES = [
    'GRASS',
    'CONCRETE', 
    'ASPHALT',
    'DIRT',
    'WATER',
    'TREE/CANOPY',
    'BUILDING/ROOF',
]

VIS_COLORS = {
    'GRASS': '#00FF00',
    'CONCRETE': '#C8C8C8',
    'ASPHALT': '#323232',
    'DIRT': '#8B4513',
    'WATER': '#0000FF',
    'TREE/CANOPY': '#006400',
    'BUILDING/ROOF': '#FF0000',
}


class SampleLabeler:
    def __init__(self, image_path, output_path):
        self.image_path = image_path
        self.output_path = output_path
        self.pil_image = Image.open(image_path).convert('RGB')
        self.img_width, self.img_height = self.pil_image.size
        
        # Load existing samples if present
        self.samples = {t: [] for t in TERRAIN_TYPES}
        if os.path.exists(output_path):
            self._load_existing()
        
        self.current_terrain = 'GRASS'
        self.drag_start = None
        self.drag_rect_id = None
        
    def _load_existing(self):
        """Load existing training_samples.json."""
        try:
            with open(self.output_path, 'r') as f:
                data = json.load(f)
            for group in data.get('samples', []):
                terrain = group['terrain']
                if terrain in self.samples:
                    self.samples[terrain].extend(group.get('regions', []))
            print(f"Loaded existing samples from {self.output_path}")
            for t, regions in self.samples.items():
                if regions:
                    print(f"  {t}: {len(regions)} regions")
        except (json.JSONDecodeError, IOError):
            pass
    
    def save(self):
        """Save samples to JSON."""
        data = {
            "description": f"Training samples for {os.path.basename(self.image_path)}",
            "samples": []
        }
        for terrain, regions in self.samples.items():
            if regions:
                data["samples"].append({
                    "terrain": terrain,
                    "regions": regions
                })
        
        with open(self.output_path, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"\nSaved {sum(len(r) for r in self.samples.values())} regions to {self.output_path}")
    
    def run_tkinter(self):
        """Run the interactive labeler using tkinter."""
        self.root = tk.Tk()
        self.root.title(f"TerrainPlan Sample Labeler - {os.path.basename(self.image_path)}")
        
        # Scale image if too large for screen
        screen_w = self.root.winfo_screenwidth() - 200
        screen_h = self.root.winfo_screenheight() - 200
        self.scale = min(1.0, screen_w / self.img_width, screen_h / self.img_height)
        
        display_w = int(self.img_width * self.scale)
        display_h = int(self.img_height * self.scale)
        
        # Frame for buttons
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        tk.Label(btn_frame, text="Terrain type:").pack(side=tk.LEFT, padx=5)
        
        self.terrain_var = tk.StringVar(value=self.current_terrain)
        for i, terrain in enumerate(TERRAIN_TYPES):
            color = VIS_COLORS.get(terrain, '#FF00FF')
            btn = tk.Radiobutton(
                btn_frame, text=f"{i+1}. {terrain}", 
                variable=self.terrain_var, value=terrain,
                command=lambda t=terrain: self._set_terrain(t),
                fg=color if color != '#C8C8C8' else '#808080',
                font=('Arial', 9, 'bold')
            )
            btn.pack(side=tk.LEFT, padx=2)
        
        # Action buttons
        action_frame = tk.Frame(self.root)
        action_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=2)
        
        tk.Button(action_frame, text="Undo Last (U)", command=self._undo).pack(side=tk.LEFT, padx=5)
        tk.Button(action_frame, text="Save & Quit (S)", command=self._save_and_quit, 
                  bg='#4CAF50', fg='white').pack(side=tk.LEFT, padx=5)
        tk.Button(action_frame, text="Quit (Q)", command=self._quit).pack(side=tk.LEFT, padx=5)
        
        self.status_var = tk.StringVar(value="Click and drag to draw a region. Select terrain type above.")
        tk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W).pack(
            side=tk.BOTTOM, fill=tk.X)
        
        # Canvas
        self.canvas = tk.Canvas(self.root, width=display_w, height=display_h)
        self.canvas.pack(side=tk.TOP, padx=5, pady=5)
        
        # Display image
        display_img = self.pil_image.resize((display_w, display_h), Image.LANCZOS)
        import tkinter as tk_  # for PhotoImage
        self.tk_image = tk.PhotoImage(data=self._pil_to_ppm(display_img))
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        
        # Draw existing regions
        self._redraw_regions()
        
        # Mouse bindings
        self.canvas.bind('<ButtonPress-1>', self._on_press)
        self.canvas.bind('<B1-Motion>', self._on_drag)
        self.canvas.bind('<ButtonRelease-1>', self._on_release)
        
        # Keyboard shortcuts
        self.root.bind('<s>', lambda e: self._save_and_quit())
        self.root.bind('<q>', lambda e: self._quit())
        self.root.bind('<u>', lambda e: self._undo())
        for i, terrain in enumerate(TERRAIN_TYPES):
            self.root.bind(str(i+1), lambda e, t=terrain: self._set_terrain(t))
        
        self.root.mainloop()
    
    def _pil_to_ppm(self, img):
        """Convert PIL image to PPM format for tkinter."""
        import io
        buf = io.BytesIO()
        img.save(buf, format='PPM')
        return buf.getvalue()
    
    def _set_terrain(self, terrain):
        self.current_terrain = terrain
        self.terrain_var.set(terrain)
        self.status_var.set(f"Selected: {terrain} — Click and drag to draw a region.")
    
    def _on_press(self, event):
        self.drag_start = (event.x, event.y)
    
    def _on_drag(self, event):
        if self.drag_start:
            if self.drag_rect_id:
                self.canvas.delete(self.drag_rect_id)
            color = VIS_COLORS.get(self.current_terrain, '#FF00FF')
            self.drag_rect_id = self.canvas.create_rectangle(
                self.drag_start[0], self.drag_start[1], event.x, event.y,
                outline=color, width=2
            )
    
    def _on_release(self, event):
        if self.drag_start:
            if self.drag_rect_id:
                self.canvas.delete(self.drag_rect_id)
                self.drag_rect_id = None
            
            # Convert display coordinates to image coordinates
            x0 = int(min(self.drag_start[0], event.x) / self.scale)
            y0 = int(min(self.drag_start[1], event.y) / self.scale)
            x1 = int(max(self.drag_start[0], event.x) / self.scale)
            y1 = int(max(self.drag_start[1], event.y) / self.scale)
            
            w = x1 - x0
            h = y1 - y0
            
            if w >= 3 and h >= 3:  # minimum region size
                region = {"x": x0, "y": y0, "w": w, "h": h}
                self.samples[self.current_terrain].append(region)
                self._redraw_regions()
                
                total = sum(len(r) for r in self.samples.values())
                self.status_var.set(
                    f"Added {self.current_terrain} region at ({x0},{y0}) {w}x{h}. "
                    f"Total regions: {total}"
                )
            
            self.drag_start = None
    
    def _redraw_regions(self):
        """Redraw all labeled regions on the canvas."""
        self.canvas.delete('region')
        for terrain, regions in self.samples.items():
            color = VIS_COLORS.get(terrain, '#FF00FF')
            for region in regions:
                x0 = int(region['x'] * self.scale)
                y0 = int(region['y'] * self.scale)
                x1 = int((region['x'] + region['w']) * self.scale)
                y1 = int((region['y'] + region['h']) * self.scale)
                
                self.canvas.create_rectangle(x0, y0, x1, y1, outline=color, width=2, tags='region')
                self.canvas.create_text(x0 + 2, y0 + 2, text=terrain[:4], anchor=tk.NW,
                                       fill=color, font=('Arial', 8, 'bold'), tags='region')
    
    def _undo(self):
        """Remove the last added region."""
        if self.samples[self.current_terrain]:
            removed = self.samples[self.current_terrain].pop()
            self._redraw_regions()
            self.status_var.set(f"Undid last {self.current_terrain} region.")
        else:
            self.status_var.set(f"No {self.current_terrain} regions to undo.")
    
    def _save_and_quit(self):
        self.save()
        self.root.destroy()
    
    def _quit(self):
        self.root.destroy()


def main():
    # Determine image path
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        for ext in ['map.tiff', 'map.tif', 'map.png']:
            candidate = os.path.join(script_dir, ext)
            if os.path.exists(candidate):
                image_path = candidate
                break
        else:
            print("No map image found. Pass the image path as an argument.")
            print("Usage: python label_samples.py path/to/map.tiff")
            return
    
    output_dir = os.path.dirname(os.path.abspath(image_path))
    output_path = os.path.join(output_dir, "training_samples.json")
    
    print(f"Image: {image_path}")
    print(f"Output: {output_path}")
    
    if not HAS_TK:
        print("\ntkinter is not available. Cannot run interactive labeler.")
        print("Please create training_samples.json manually (see format in README).")
        print("You can use any image editor to find pixel coordinates.")
        return
    
    labeler = SampleLabeler(image_path, output_path)
    labeler.run_tkinter()


if __name__ == "__main__":
    main()
