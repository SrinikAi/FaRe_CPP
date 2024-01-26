from FireBotMAP import Map_generator

map_generator = Map_generator()
def process_frontier(graph,frontier, scout, surveillance_range, free_cells, yaml_data, state):
    sub_graph = scout.increment_cells_within_circle(graph,frontier, surveillance_range, free_cells)
    area = map_generator.estimate_area(sub_graph, yaml_data, state)  # Assuming map_generator is defined
    return {
        'frontier': frontier,
        'area': area,
        'sub_graph': sub_graph
    }
    pass