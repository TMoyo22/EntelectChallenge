import ast
import re
import math

# Step 1: Zoo file content
zoo_file_contents = """(100,100,50)
(50,49,19)
999999
[(56,74,13,h),(38,79,11,c),(20,49,37,o)]
[(88,78,14,0.09,h),(36,67,35,10,c),(72,4,27,1.49,o),(14,21,26,8.98,o),(27,36,27,0.34,o),(68,35,19,18.81,o),(25,15,34,11.28,h),(40,17,19,8,c),(55,96,9,3.6,h),(88,48,2,1.23,h),(72,67,37,3.53,h),(72,88,16,15,c),(5,34,33,6,c),(25,82,38,5.0,o),(84,12,25,2.99,o),(49,84,14,3.49,o),(32,51,2,6.59,o),(26,45,20,11.66,h),(37,59,35,1.12,o),(33,82,9,3,c)]
[]"""

# Step 2: Helper to quote diet letters
def quote_diets(line):
    return re.sub(r'(?<=[,(])\s*([hco])\s*(?=[,\)])', r"'\1'", line)

# Step 3: Parse function
def parse_zoo_string(content):
    lines = content.strip().split('\n')
    lines[3] = quote_diets(lines[3])
    lines[4] = quote_diets(lines[4])
    
    zoo_dimensions = ast.literal_eval(lines[0])
    drone_depot = ast.literal_eval(lines[1])
    battery_capacity = int(lines[2])
    food_storages = ast.literal_eval(lines[3])
    enclosures = ast.literal_eval(lines[4])
    deadzones = ast.literal_eval(lines[5]) if len(lines) > 5 else []

    return {
        "zoo_dimensions": zoo_dimensions,
        "drone_depot": drone_depot,
        "battery_capacity": battery_capacity,
        "food_storages": food_storages,
        "enclosures": enclosures,
        "deadzones": deadzones
    }

# Step 4: Distance calculator (2D only for pathing)
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# Step 5: Find matching food storages
def find_food_storage(storages, diet):
    return [s for s in storages if s[3] == diet]

# Step 6: Greedy drone path planner
def plan_drone_path(parsed_data):
    depot = parsed_data["drone_depot"]
    food_storages = parsed_data["food_storages"]
    enclosures = parsed_data["enclosures"]

    path = [depot[:2]]  # 2D path
    carrying = None
    fed_enclosures = set()

    # Sort enclosures by importance
    sorted_enclosures = sorted(enclosures, key=lambda e: -e[3])

    for enc in sorted_enclosures:
        x, y, z, importance, diet = enc

        if ((x, y) in fed_enclosures):
            continue

        # Switch food if needed
        if carrying != diet:
            matching_storages = find_food_storage(food_storages, diet)
            nearest_storage = min(matching_storages, key=lambda s: distance(path[-1], s[:2]))
            path.append(nearest_storage[:2])
            carrying = diet

        # Feed enclosure
        path.append((x, y))
        fed_enclosures.add((x, y))

    # Return to depot
    path.append(depot[:2])
    return path

# Run everything
parsed_data = parse_zoo_string(zoo_file_contents)
drone_path = plan_drone_path(parsed_data)

# Final output for Level 1
submission = [drone_path]

# Print the result
print("📝 Drone Path Submission:")
print(submission)

