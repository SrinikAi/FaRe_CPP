import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import yaml
import time
from scipy.spatial.distance import pdist, squareform
from itertools import permutations
import multiprocessing 
from numba import jit
# Assuming FireBotMAP and Multi_Processing modules are available in your environment.
from FireBotMAP import Map_generator
from Multi_Processing import process_frontier

# Initialize the map generator
map_generator = Map_generator()

@jit(nopython=True)
def bresenhams_line_algorithm(x0, y0, x1, y1):
    points = []
    dx = x1 - x0
    dy = y1 - y0
    is_steep = abs(dy) > abs(dx)
    if is_steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    swapped = False
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
        swapped = True
    dx = x1 - x0
    dy = y1 - y0
    error = dx / 2.0
    ystep = 1 if y0 < y1 else -1
    y = y0
    for x in range(x0, x1 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    if swapped:
        points.reverse()
    return points

class Scout:
    def __init__(self):
        pass

    def get_circle_perimeter_points(self, center, radius, num_points=360):
        perimeter_points = set()
        for theta in np.linspace(0, 2 * np.pi, num=num_points):
            x = int(radius * np.cos(theta) + center[0])
            y = int(radius * np.sin(theta) + center[1])
            perimeter_points.add((x, y))
        return list(perimeter_points)

    
    def line_of_sight(self, grid_map, start, end, unoccupied_value):
        points = bresenhams_line_algorithm(start[0], start[1], end[0], end[1])
        return all(grid_map[x, y] == unoccupied_value for x, y in points)

    def increment_cells_within_circle(self, grid_map, start_position, radius, unoccupied_value):
        
        
        assert grid_map[start_position] == unoccupied_value, "Starting position must be in unoccupied space."
        
        updated_map = np.copy(grid_map) 
        rows, cols = grid_map.shape

        for i in range(rows):
            for j in range(cols):
                if self.is_within_circle(start_position, (i, j), radius):
                    if grid_map[i, j] == unoccupied_value and self.line_of_sight(grid_map, start_position, (i, j), unoccupied_value):
                        if grid_map[i, j] != 255:
                            updated_map[i, j] += 1
        
        return updated_map
    def is_within_circle(self, center, point, radius):
        return np.linalg.norm(np.array(center) - np.array(point)) <= radius
    def find_frontier_cells(self, grid_map, explored_value, unexplored_value):
        rows, cols = grid_map.shape
        frontier_cells = []

        for i in range(rows):
            for j in range(cols):
                
                if grid_map[i, j] == unexplored_value:
                    
                    if ((i > 0 and grid_map[i-1, j] == explored_value) or
                        (i < rows - 1 and grid_map[i+1, j] == explored_value) or
                        (j > 0 and grid_map[i, j-1] == explored_value) or
                        (j < cols - 1 and grid_map[i, j+1] == explored_value)):
                        frontier_cells.append((i, j))
        
        return frontier_cells
    
class Exploration:
    def __init__(self, grid_map, surveillance_range, free_cells, state, yaml_data):
        self.grid_map = grid_map
        self.surveillance_range = surveillance_range
        self.free_cells = free_cells
        self.state = state
        self.yaml_data = yaml_data
        self.scout = Scout() 

    def surveillance(self, iteration, frontiers, graph, area):
        
        with multiprocessing.Pool() as pool:
            # Prepare arguments for each process
            args = [(graph, frontier, self.scout, self.surveillance_range, self.free_cells, self.yaml_data, self.state) for frontier in frontiers]

            # Execute the function in parallel
            results = pool.starmap(process_frontier, args)

        
        max_area_dict = max(results, key=lambda x: x['area'])
        selected_frontier, area, graph = max_area_dict['frontier'], max_area_dict['area'], max_area_dict['sub_graph']
        return selected_frontier, area, graph
    def set_goals(self, current_pos, explored_value, unexplored_value,steps, frontier_drop_rate):
        total_area = 0
        iteration = 0
        graph = self.grid_map
        area_goals = []

        for i in range(steps):
            start_time = time.time()
            frontiers = current_pos if i == 0 else self.scout.find_frontier_cells(graph, explored_value, unexplored_value)
            if frontier_drop_rate > 0:
                frontiers = [item for index, item in enumerate(frontiers) if index == 0 or (index + 1) % frontier_drop_rate == 0]
            if not frontiers:
                print("No more frontiers found. Stopping exploration.")
                break
            selected_frontier, area, updated_graph = self.surveillance(iteration, frontiers, graph, total_area)
            total_area = area 
            iteration += 1 
            graph = updated_graph
            end_time = time.time()
            area_goals.append({'iteration': iteration, 'goals': {'goal': selected_frontier, 'area': area}, 'graph':graph})
            
            print(f'iteration: {iteration} goal : {selected_frontier}   explored area : {area} frontiers: {len(frontiers)} excution_time: {int(end_time - start_time)} seconds ' )
            
        return area_goals
    def optimize_goals(self,goal_points):
   
        points = [item['goals']['goal'] for item in goal_points]

        
        distance_matrix = squareform(pdist(points, 'euclidean'))

        
        def total_distance(path):
            return sum(distance_matrix[path[i], path[i+1]] for i in range(len(path) - 1))

        
        all_paths = permutations(range(1, len(points)))
        min_path = None
        min_distance = float('inf')

       
        for path in all_paths:
            current_path = (0,) + path + (0,)
            current_distance = total_distance(current_path)
            if current_distance < min_distance:
                min_distance = current_distance
                min_path = current_path

        
        min_path_points = [points[i] for i in min_path]

      
        coord_to_iter_index = {item['goals']['goal']: idx for idx, item in enumerate(goal_points)}

       
        ordered_goals = [goal_points[coord_to_iter_index[point]] for point in min_path_points[1:-1]]

        return min_path_points, min_distance, ordered_goals