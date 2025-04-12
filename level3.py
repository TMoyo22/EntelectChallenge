import heapq
import math

def a_star(start, goal, obstacles, dimensions, dynamic_obstacles, current_time):
    """
    A* pathfinding algorithm to find the shortest path from start to goal, avoiding dynamic obstacles.
    :param start: Starting point (x, y)
    :param goal: Goal point (x, y)
    :param obstacles: Set of static obstacle points (x, y)
    :param dimensions: Dimensions of the grid (width, height)
    :param dynamic_obstacles: List of dynamic obstacles (x, y, start_time, end_time)
    :param current_time: Current time step
    :return: List of points representing the path from start to goal
    """
    def is_obstacle(node, time):
        # Check if the node is a static or dynamic obstacle at the given time
        if node in obstacles:
            return True
        for obs in dynamic_obstacles:
            x, y, z, start_time, end_time = obs
            if (x, y) == node and start_time <= time <= end_time:
                return True
        return False
    
    def heuristic(node, goal):
        """
        Calculate the Manhattan distance between the current node and the goal.
        :param node: Current node (x, y)
        :param goal: Goal node (x, y)
        :return: Manhattan distance
     """
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def neighbors(node):
        # Generate valid neighbors (up, down, left, right)
        x, y = node
        potential_neighbors = [
            (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)
        ]
        # Filter neighbors within bounds and not in obstacles
        return [
            (nx, ny) for nx, ny in potential_neighbors
            if 0 <= nx < dimensions[0] and 0 <= ny < dimensions[1] and not is_obstacle((nx, ny), current_time)
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