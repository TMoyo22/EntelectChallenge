import ast
import math
from collections import defaultdict

# === CONFIG ===
INPUT_FILE = "4.txt"
MAX_RUNS = 250
MAX_DISTANCE = 9250

# === HELPERS ===
def extract_bracketed_list(text, start_index):
    bracket_count = 0
    start = text.find('[', start_index)
    for i in range(start, len(text)):
        if text[i] == '[':
            bracket_count += 1
        elif text[i] == ']':
            bracket_count -= 1
            if bracket_count == 0:
                return text[start:i + 1], i + 1
    raise ValueError("Unmatched brackets")

def clean_list_string(raw):
    raw = raw.replace('\n', '').replace('),(', ')####(')
    raw = raw.strip()[1:-1]  # remove outer []
    parts = raw.split('####')
    return [ast.literal_eval(p) for p in parts]

def euclidean_3d(p1, p2):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

def vertical_distance(p1, target_z=50):
    return abs(target_z - p1[2])

def drone_trip_distance(path, depot_z):
    if not path or len(path) < 2:
        return 0
    total = 0
    total += vertical_distance((*path[0], depot_z))  # takeoff
    for i in range(len(path) - 1):
        total += euclidean_3d((*path[i], 50), (*path[i+1], 50))
    total += sum(vertical_distance((*pt, 0)) * 2 for pt in path[1:-1])
    total += vertical_distance((*path[-1], depot_z))  # final landing
    return total

# === MAIN ===
def main():
    with open(INPUT_FILE, "r") as file:
        full_text = file.read()

    lines = full_text.strip().split('\n')
    drone_depot = ast.literal_eval(lines[1].strip())

    # Parse lists
    food_str, next_index = extract_bracketed_list(full_text, 0)
    food_storages = clean_list_string(food_str)

    enclosures_str, _ = extract_bracketed_list(full_text, next_index)
    enclosures = clean_list_string(enclosures_str)

    # Organize by diet
    storage_by_diet = defaultdict(list)
    for x, y, z, diet in food_storages:
        storage_by_diet[diet].append((x, y, z))

    enclosures_by_diet = defaultdict(list)
    for x, y, z, importance, diet in enclosures:
        enclosures_by_diet[diet].append(((x, y, z), importance))

    for diet in enclosures_by_diet:
        enclosures_by_diet[diet].sort(key=lambda x: -x[1])  # importance desc

    used = set()
    all_runs = []

    for _ in range(MAX_RUNS):
        best_route, best_score = None, float('-inf')
        for diet in ['c', 'h', 'o']:
            if not enclosures_by_diet[diet]:
                continue
            for storage in storage_by_diet[diet]:
                route = [(drone_depot[0], drone_depot[1]), (storage[0], storage[1])]
                importance_sum = 0
                temp_route = list(route)
                temp_used = set()
                for (x, y, z), imp in enclosures_by_diet[diet]:
                    if (x, y) in used or (x, y) in temp_used:
                        continue
                    test_route = temp_route[:-1] + [(x, y), (drone_depot[0], drone_depot[1])]
                    d = drone_trip_distance(test_route, drone_depot[2])
                    if d <= MAX_DISTANCE:
                        temp_route.insert(-1, (x, y))
                        temp_used.add((x, y))
                        importance_sum += imp
                    else:
                        break
                score = importance_sum * 1000 - drone_trip_distance(temp_route, drone_depot[2])
                if score > best_score:
                    best_score = score
                    best_route = (temp_route, temp_used)
        if not best_route:
            break
        all_runs.append(best_route[0])
        used.update(best_route[1])
        for diet in ['c', 'h', 'o']:
            enclosures_by_diet[diet] = [e for e in enclosures_by_diet[diet] if (e[0][0], e[0][1]) not in used]

    # Print final submission format
    print("[")
    for i, run in enumerate(all_runs):
        print(f"  {run}," if i < len(all_runs) - 1 else f"  {run}")
    print("]")

if __name__ == "__main__":
    main()
