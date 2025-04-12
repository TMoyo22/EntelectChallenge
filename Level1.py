import heapq

# A* Algorithm for shortest path
def a_star(start, goal, obstacles, dimensions):
    """
    A* pathfinding algorithm to find the shortest path from start to goal.
    :param start: Starting point (x, y)
    :param goal: Goal point (x, y)
    :param obstacles: Set of obstacle points (x, y)
    :param dimensions: Dimensions of the grid (width, height)
    :return: List of points representing the path from start to goal
    """
    def heuristic(p1, p2):
        # Heuristic: Euclidean distance
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def neighbors(node):
        # Generate valid neighbors (up, down, left, right)
        x, y = node
        potential_neighbors = [
            (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)
        ]
        # Filter neighbors within bounds and not in obstacles
        return [
            (nx, ny) for nx, ny in potential_neighbors
            if 0 <= nx < dimensions[0] and 0 <= ny < dimensions[1] and (nx, ny) not in obstacles
        ]

    # Priority queue for open nodes
    open_set = []
    heapq.heappush(open_set, (0, start))

    # Maps to store the cost and path
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    came_from = {}

    while open_set:
        # Get the node with the lowest f_score
        _, current = heapq.heappop(open_set)

        # If we reached the goal, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Reverse the path

        # Explore neighbors
        for neighbor in neighbors(current):
            tentative_g_score = g_score[current] + 1  # Assume uniform cost for moving

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # Update scores and path
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                # Add neighbor to open set
                if neighbor not in [item[1] for item in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # If no path is found
    return []

# Updated drone path planner using A*
def plan_drone_path_with_astar(parsed_data):
    depot = parsed_data["drone_depot"]
    food_storages = parsed_data["food_storages"]
    enclosures = parsed_data["enclosures"]
    deadzones = set((dz[0], dz[1]) for dz in parsed_data["deadzones"])
    dimensions = parsed_data["zoo_dimensions"][:2]

    # Preprocess food storages by diet
    food_storage_map = preprocess_food_storages(food_storages)

    path = []
    carrying = None
    fed_enclosures = set()

    # Sort enclosures by importance (descending)
    sorted_enclosures = sorted(enclosures, key=lambda e: -e[3])

    current_position = depot[:2]
    path.append(current_position)

    for enc in sorted_enclosures:
        x, y, z, importance, diet = enc

        if (x, y) in fed_enclosures:
            continue

        # Switch food if needed
        if carrying != diet:
            matching_storages = food_storage_map[diet]
            nearest_storage = min(matching_storages, key=lambda s: distance(current_position, s[:2]))
            storage_path = a_star(current_position, nearest_storage[:2], deadzones, dimensions)
            path.extend(storage_path[1:])  # Avoid duplicating the current position
            current_position = nearest_storage[:2]
            carrying = diet

        # Feed enclosure
        enclosure_path = a_star(current_position, (x, y), deadzones, dimensions)
        path.extend(enclosure_path[1:])  # Avoid duplicating the current position
        current_position = (x, y)
        fed_enclosures.add((x, y))

    # Return to depot
    return_to_depot_path = a_star(current_position, depot[:2], deadzones, dimensions)
    path.extend(return_to_depot_path[1:])  # Avoid duplicating the current position

    return path

# Run everything with A*
parsed_data = parse_zoo_string(zoo_file_contents)
drone_path = plan_drone_path_with_astar(parsed_data)

# Final output for Level 1
submission = [drone_path]

# Print the result
print("📝 Drone Path Submission with A*: ")
print(submission)