
import numpy as np
import matplotlib.pyplot as plt
import yaml
from PIL import Image
from FireBot_path_optimizer import WayPointOptimizer


goals = [
    (350, 67), (319, 93), (332, 131), (325, 171), (285, 174), (244, 174), (239, 214),
    (247, 134), (226, 252), (185, 252), (241, 290), (230, 329), (266, 348), (305, 338),
    (247, 93), (323, 374), (317, 299), (164, 287), (264, 388), (226, 403), (289, 211),
    (329, 217), (249, 152), (154, 326), (310, 141), (265, 276), (294, 297), (206, 320),
    (199, 355), (193, 213), (302, 384), (297, 412), (221, 371), (347, 123), (259, 215),
    (345, 332), (258, 333), (344, 363), (146, 248), (179, 207)
]

optimized_goals = grasp_solver = WayPointOptimizer(goals,0.3,5)

best_path = grasp_solver.run()

print(best_path)

def load_pgm(pgm_path):
        with Image.open(pgm_path) as img:
            return np.array(img)

def load_yaml(yaml_file_path):
    """Load YAML file and return the content."""
    with open(yaml_file_path, 'r') as file:
        return yaml.safe_load(file)

def heuristic(a, b):
    """Calculate the Manhattan distance between two points."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(map_data, start, goal, free_space=[254]):
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4-way connectivity
    open_set = {start}
    came_from = {}
    
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, np.inf))
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)  # Optional: include start in path
            return path[::-1]  # Return reversed path
        
        open_set.remove(current)
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check if within bounds and navigable
            if 0 <= neighbor[0] < map_data.shape[0] and 0 <= neighbor[1] < map_data.shape[1]:
                if map_data[neighbor[0], neighbor[1]] not in free_space:
                    continue
                
                tentative_g_score = g_score[current] + 1  # Cost = 1 per step
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.add(neighbor)
    
    return [] 
    
def convert_pgm_to_binary_custom(image_array):
    """
    Convert a .pgm file to a binary .png file where value 254 is converted to white (255)
    and all other values to black (0).

    Parameters:
    - pgm_file_path: str, path to the .pgm file
    - png_file_path: str, path where the .png file will be saved
    """
    # Load the PGM file
   

    # Convert value 254 to white (255), and all other values to black (0)
    binary_image_array = np.where(image_array == 254, 255, 0).astype(np.uint8)

    # Convert the modified NumPy array back to an image
    binary_image = Image.fromarray(binary_image_array)    
    return binary_image
    
    
map_data = load_pgm(r"D:\srinika\Research_Track\house_ipa_results\map\map.pgm")
metadata = load_yaml(r"D:\srinika\Research_Track\house_ipa_results\map\map.yaml")

print(np.unique(map_data))
resolution = metadata['resolution']
origin = metadata['origin'][:2]  # Only need x and y

# Transform goal coordinates (these are placeholders, replace with actual transformation based on map)
goals_transformed = [
    # Placeholder transformation; replace with the actual transformation logic based on map's resolution and origin
]

goals = best_path
total_path_length = 0
fig, ax = plt.subplots(figsize=(10, 10))
b_map = convert_pgm_to_binary_custom(map_data)
ax.imshow(b_map, cmap='gray')

for i in range(len(goals) - 1):
    path = a_star(map_data, goals[i], goals[i+1])
    if path:
        # Assuming resolution is the distance in meters for each grid cell
        total_path_length += len(path) * resolution
        # Plotting the path
        y, x = zip(*path)
        ax.plot(x, y, '-r')

# Plotting goals
for goal in goals:
    ax.plot(goal[1], goal[0], 'b*', markersize=5)

plt.show()


plt.show()
print(total_path_length)