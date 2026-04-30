# core/visualizer.py
import threading
import tkinter as tk
import math

TRACK_WIDTH = 10.767
TRACK_HEIGHT = 10.767
SVG_SIZE = 10000
SCALE = SVG_SIZE / TRACK_WIDTH

WINDOW_SIZE = 800

class TrackVisualizer(threading.Thread):
    def __init__(self, world):
        super().__init__()
        self.world = world
        self.running = True
        self.robot_x = 0
        self.robot_y = 0
        self.robot_h = 0
        
    def run(self):
        self.root = tk.Tk()
        self.root.title("Robot Track Visualizer")
        
        self.canvas = tk.Canvas(
            self.root, 
            width=WINDOW_SIZE, 
            height=WINDOW_SIZE, 
            bg="#292929"
        )
        self.canvas.pack()
        
        self.draw_track()
        self.robot_item = None
        
        self.update()
        self.root.mainloop()
    
    def draw_track(self):
        svg_path = "core/track_texture.svg"
        
        try:
            with open(svg_path, 'r') as f:
                content = f.read()
            
            import re
            
            white_paths = re.findall(r'd="([^"]+)"[^>]*stroke:#ffffff', content)
            
            for path_data in white_paths:
                points = self.parse_svg_path(path_data)
                if len(points) >= 2:
                    scaled_points = [(self.world_to_screen(x, y)) for x, y in points]
                    flat_points = [coord for point in scaled_points for coord in point]
                    self.canvas.create_line(*flat_points, fill="white", width=2)
                    
        except Exception as e:
            print(f"Could not load track: {e}")
            self.draw_grid()
    
    def parse_svg_path(self, path_data):
        import re
        points = []
        
        commands = re.findall(r'([MLHVCSQTAZ])([^MLHVCSQTAZ]*)', path_data)
        
        current_x, current_y = 0, 0
        
        for cmd, args in commands:
            args = args.strip()
            if not args:
                continue
                
            nums = [float(n) for n in re.findall(r'-?\d+\.?\d*', args)]
            
            if cmd == 'M' or cmd == 'L':
                for i in range(0, len(nums), 2):
                    if i + 1 < len(nums):
                        current_x, current_y = nums[i], nums[i + 1]
                        points.append((current_x, current_y))
            elif cmd == 'H':
                for n in nums:
                    current_x = n
                    points.append((current_x, current_y))
            elif cmd == 'V':
                for n in nums:
                    current_y = n
                    points.append((current_x, current_y))
            elif cmd == 'm' or cmd == 'l':
                for i in range(0, len(nums), 2):
                    if i + 1 < len(nums):
                        current_x += nums[i]
                        current_y += nums[i + 1]
                        points.append((current_x, current_y))
            elif cmd == 'h':
                for n in nums:
                    current_x += n
                    points.append((current_x, current_y))
            elif cmd == 'v':
                for n in nums:
                    current_y += n
                    points.append((current_x, current_y))
            elif cmd == 'C':
                for i in range(0, len(nums), 6):
                    if i + 5 < len(nums):
                        current_x, current_y = nums[i + 4], nums[i + 5]
                        points.append((current_x, current_y))
            elif cmd == 'c':
                for i in range(0, len(nums), 6):
                    if i + 5 < len(nums):
                        current_x += nums[i + 4]
                        current_y += nums[i + 5]
                        points.append((current_x, current_y))
            elif cmd == 'z' or cmd == 'Z':
                pass
                
        return points
    
    def draw_grid(self):
        spacing = WINDOW_SIZE / 10
        for i in range(11):
            pos = i * spacing
            self.canvas.create_line(pos, 0, pos, WINDOW_SIZE, fill="#444444", dash=(2, 4))
            self.canvas.create_line(0, pos, WINDOW_SIZE, pos, fill="#444444", dash=(2, 4))
    
    def world_to_screen(self, x, y):
        screen_x = (x / SVG_SIZE) * WINDOW_SIZE
        screen_y = WINDOW_SIZE - (y / SVG_SIZE) * WINDOW_SIZE
        return screen_x, screen_y
    
    def update(self):
        if not self.running:
            return
            
        pose = self.world.get_pose()
        self.robot_x, self.robot_y, self.robot_h = pose
        
        screen_x, screen_y = self.world_to_screen(
            self.robot_x * SCALE, 
            self.robot_y * SCALE
        )
        
        if self.robot_item:
            self.canvas.delete(self.robot_item)
        
        arrow_len = 20
        end_x = screen_x + arrow_len * math.cos(self.robot_h)
        end_y = screen_y - arrow_len * math.sin(self.robot_h)
        
        self.robot_item = self.canvas.create_oval(
            screen_x - 8, screen_y - 8,
            screen_x + 8, screen_y + 8,
            fill="#00ff00", outline="white", width=2
        )
        
        self.canvas.create_line(
            screen_x, screen_y, end_x, end_y,
            fill="#00ff00", width=3
        )
        
        self.root.after(50, self.update)
    
    def stop(self):
        self.running = False
        try:
            self.root.quit()
        except:
            pass
