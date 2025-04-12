import heapq
import math

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
    
        

    def preprocess_food_storages(food_storages):
        """
            Preprocess food storages by grouping them based on diet type.
            :param food_storages: List of food storages (x, y, z, capacity, diet)
            :return: Dictionary mapping diet type to a list of food storages
    """
        food_storage_map = {}
        for storage in food_storages:
                x, y, z, capacity, diet = storage
                if diet not in food_storage_map:
                    food_storage_map[diet] = []
                    food_storage_map[diet].append(storage)
        return food_storage_map
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

def distance(point1, point2):
    """
    Calculate the Euclidean distance between two points in 2D space.
    :param point1: Tuple (x1, y1)
    :param point2: Tuple (x2, y2)
    :return: Euclidean distance between point1 and point2
    """
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Parse the new input format
def parse_level2_input(zoo_size, enclosures, food_storages, battery_swaps):
    """
    Parse the input for Level 2.
    :param zoo_size: Tuple (width, height, depth)
    :param enclosures: List of enclosures (x, y, z, importance, diet)
    :param food_storages: List of food storages (x, y, z, capacity, diet)
    :param battery_swaps: List of battery swap stations (x, y, z)
    :return: Parsed data dictionary
    """
    parsed_data = {
        "zoo_dimensions": zoo_size,
        "enclosures": enclosures,
        "food_storages": food_storages,
        "battery_swaps": battery_swaps
    }
    return parsed_data

# Plan the drone path for Level 2
def plan_drone_path_level2(parsed_data):
    depot = (0, 0, 0)  # Assume the depot is at (0, 0, 0)
    enclosures = parsed_data["enclosures"]
    food_storages = parsed_data["food_storages"]
    battery_swaps = parsed_data["battery_swaps"]
    dimensions = parsed_data["zoo_dimensions"][:2]
    battery_capacity = 1125

    # Preprocess food storages by diet
    food_storage_map = preprocess_food_storages(food_storages)

    path = []
    carrying = None
    fed_enclosures = set()
    remaining_battery = battery_capacity

    # Sort enclosures by importance (descending)
    sorted_enclosures = sorted(enclosures, key=lambda e: -e[3])

    current_position = depot[:2]
    path.append(current_position)

    for enc in sorted_enclosures:
        x, y, z, importance, diet = enc

        if (x, y) in fed_enclosures:
            continue

        # Check if battery swap is needed
        if remaining_battery < distance(current_position, (x, y)):
            nearest_battery_swap = min(battery_swaps, key=lambda b: distance(current_position, b[:2]))
            battery_path = a_star(current_position, nearest_battery_swap[:2], set(), dimensions)
            path.extend(battery_path[1:])  # Avoid duplicating the current position
            current_position = nearest_battery_swap[:2]
            remaining_battery = battery_capacity

        # Switch food if needed
        if carrying != diet:
            matching_storages = food_storage_map[diet]
            nearest_storage = min(matching_storages, key=lambda s: distance(current_position, s[:2]))
            storage_path = a_star(current_position, nearest_storage[:2], set(), dimensions)
            path.extend(storage_path[1:])  # Avoid duplicating the current position
            current_position = nearest_storage[:2]
            carrying = diet

        # Feed enclosure
        enclosure_path = a_star(current_position, (x, y), set(), dimensions)
        path.extend(enclosure_path[1:])  # Avoid duplicating the current position
        current_position = (x, y)
        fed_enclosures.add((x, y))
        remaining_battery -= distance(current_position, (x, y))

    # Return to depot
    return_to_depot_path = a_star(current_position, depot[:2], set(), dimensions)
    path.extend(return_to_depot_path[1:])  # Avoid duplicating the current position

    return path

# Example usage
zoo_size = (250, 250, 50)
enclosures = [(101, 68, 1, 4.91, "h"), (189, 176, 38, 13.04, "h"), ...]  # Truncated for brevity
food_storages = [(245, 80, 45, 4.91, "o"), (142, 90, 38, 3, "c"), ...]  # Truncated for brevity
battery_swaps = []  # No battery swaps in this level

parsed_data = parse_level2_input(zoo_size, enclosures, food_storages, battery_swaps)
drone_path = plan_drone_path_level2(parsed_data)

# Final output for Level 2
submission = [drone_path]

# Print the result
print("📝 Drone Path Submission for Level 2: ")
print(submission)