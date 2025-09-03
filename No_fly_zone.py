import heapq
import matplotlib.pyplot as plt
import numpy as np

class Drone:
    def __init__(self, id, speed, capacity, max_distance, available):
        self.id = id
        self.speed = speed
        self.capacity = capacity
        self.max_distance = max_distance
        self.available = available
        self.orders = []
        self.total_distance = 0
        self.current_location = (0, 0)

class Order:
    def __init__(self, id, destination, weight, deadline):
        self.id = id
        self.destination = destination
        self.weight = weight
        self.deadline = deadline

# A* Algorithm with deviation check
def a_star(start, goal, obstacles, grid_size):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path = path[::-1]

            # Check ideal path for obstacles
            ideal_path = []
            x0, y0 = start
            x1, y1 = goal
            temp_x, temp_y = x0, y0
            while temp_x != x1:
                temp_x += 1 if x1 > temp_x else -1
                ideal_path.append((temp_x, temp_y))
            while temp_y != y1:
                temp_y += 1 if y1 > temp_y else -1
                ideal_path.append((temp_x, temp_y))

            if any(p in obstacles for p in ideal_path):
                print(f"[INFO] Obstacle detected between {start} and {goal}. Taking a deviation path.")

            return path
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < grid_size and 0 <= neighbor[1] < grid_size:
                if neighbor in obstacles:
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    print(f"[WARNING] No available path from {start} to {goal} due to heavy obstacles.")
    return []

# Grid size and obstacles
grid_size = 20
# obstacles = {(5, 5), (6, 5), (7, 5), (5, 10), (4, 11),(12,8)}
obstacles = {(5, 5), (6, 5), (7, 5),(2,9),(2,10), (2, 11),(12,8)}

# Drones
drones = [
    Drone(id='D1', speed=2, capacity=25, max_distance=35, available=True),
    Drone(id='D2', speed=1.5, capacity=10, max_distance=25, available=True),
    Drone(id='D3', speed=2.5, capacity=3, max_distance=20, available=True),
    Drone(id='D4', speed=1, capacity=20, max_distance=25, available=False)
]

# Orders
orders = [
    Order(id='O1', destination=(5, 10), weight=2, deadline=15),
    Order(id='O2', destination=(5, 3), weight=8, deadline=30),
    Order(id='O3', destination=(12, 8), weight=4, deadline=25),
    Order(id='O4', destination=(3, 2), weight=16, deadline=40)
]

# # Assign orders to drones
# def allocate_orders(drones, orders):
#     for order in orders:
#         best_drone = None
#         best_distance = float("inf")
#         for drone in drones:
#             if drone.available and order.weight <= drone.capacity:
#                 path = a_star(drone.current_location, order.destination, obstacles, grid_size)
#                 if path:
#                     distance = len(path) - 1
#                     return_path = a_star(order.destination, (0, 0), obstacles, grid_size)
#                     return_distance = len(return_path) - 1 if return_path else float("inf")
#                     total_travel = distance + return_distance

#                     if total_travel <= drone.max_distance and total_travel < best_distance:
#                         best_drone = drone
#                         best_distance = total_travel

#         if best_drone:
#             best_drone.orders.append(order.id)
#             best_drone.total_distance += best_distance
#             best_drone.current_location = order.destination
#         else:
#             print(f"[FAIL] No suitable drone found for Order {order.id}")

def allocate_orders(drones, orders):
    for order in orders:
        # Check if destination is in the obstacle set
        if order.destination in obstacles:
            print(f"Unable to deliver: No-fly zone. Order ID: {order.id}")
            continue
        
        best_drone = None
        best_distance = float("inf")
        for drone in drones:
            if drone.available and order.weight <= drone.capacity:
                path = a_star(drone.current_location, order.destination, obstacles, grid_size)
                if path:
                    distance = len(path) - 1
                    return_path = a_star(order.destination, (0, 0), obstacles, grid_size)
                    return_distance = len(return_path) - 1 if return_path else float("inf")
                    total_travel = distance + return_distance

                    if total_travel <= drone.max_distance and total_travel < best_distance:
                        best_drone = drone
                        best_distance = total_travel

        if best_drone:
            best_drone.orders.append(order.id)
            best_drone.total_distance += best_distance
            best_drone.current_location = order.destination
        else:
            print(f"Unable to assign drone for Order ID: {order.id} (No suitable drone or blocked path)")



allocate_orders(drones, orders)

# Results
print("\n[RESULT] Drone Assignments:")
for drone in drones:
    if drone.orders:
        print(f"DRONE {drone.id}: ORDERS {drone.orders}, TOTAL DISTANCE TRAVELLED {drone.total_distance}")

# Visualization
def plot_grid(grid_size, obstacles, drones, orders):
    grid = np.zeros((grid_size, grid_size))
    for obs in obstacles:
        grid[obs[1], obs[0]] = 1

    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap="gray_r", origin="upper")
    plt.scatter(0, 0, c='blue', label='Warehouse', marker='o')

    for order in orders:
        plt.scatter(order.destination[0], order.destination[1], c='green', marker='s')
        plt.text(order.destination[0], order.destination[1], order.id, fontsize=12, ha='right', va='bottom')

    colors = ["red", "blue", "orange", "purple"]
    color_map = {drone.id: colors[i % len(colors)] for i, drone in enumerate(drones)}

    for drone in drones:
        for order_id in drone.orders:
            order = next(o for o in orders if o.id == order_id)
            path = a_star((0, 0), order.destination, obstacles, grid_size)
            if path:
                path_x, path_y = zip(*path)
                plt.plot(path_x, path_y, color=color_map[drone.id], label=f'{drone.id} -> {order.id}')

    plt.legend()
    plt.grid(True)
    plt.xticks(range(grid_size))
    plt.yticks(range(grid_size))
    plt.title("Drone Delivery Paths with Obstacles")
    plt.show()

plot_grid(grid_size, obstacles, drones, orders)
