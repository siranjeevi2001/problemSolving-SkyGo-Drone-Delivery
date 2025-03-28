import heapq
import json
import matplotlib.pyplot as plt


class Drone:
    def __init__(self, id, speed, capacity, max_distance, available=True):
        self.id = id
        self.speed = speed
        self.capacity = capacity
        self.max_distance = max_distance
        self.available = available
        self.orders = []
        self.total_distance = 0
        self.current_location = (0, 0)
        self.remaining_capacity = capacity
        self.remaining_distance = max_distance
        self.travel_time = 0


class Order:
    def __init__(self, id, destination, weight, deadline):
        self.id = id
        self.destination = destination
        self.weight = weight
        self.deadline = deadline


# Manhattan distance heuristic
def calculate_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


# A* Pathfinding Algorithm
def astar_path(start, end):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: calculate_distance(start, end)}
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + calculate_distance(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []


# Assign orders to drones based on deadline priority
def assign_orders(drones, orders):
    orders.sort(key=lambda o: o.deadline)
    for order in orders:
        best_drone = None
        for drone in drones:
            if not drone.available:
                continue
            if drone.remaining_capacity >= order.weight:
                travel_distance = calculate_distance(drone.current_location, order.destination)
                return_distance = calculate_distance(order.destination, (0, 0))
                if travel_distance + return_distance <= drone.remaining_distance:
                    best_drone = drone if best_drone is None or drone.speed > best_drone.speed else best_drone

        if best_drone:
            best_drone.orders.append(order.id)
            delivery_distance = calculate_distance(best_drone.current_location, order.destination)
            return_distance = calculate_distance(order.destination, (0, 0))
            best_drone.total_distance += delivery_distance + return_distance
            best_drone.current_location = (0, 0)
            best_drone.remaining_capacity -= order.weight
            best_drone.remaining_distance -= delivery_distance + return_distance
            best_drone.travel_time += delivery_distance / best_drone.speed


# Visualize drone paths using Matplotlib
def plot_drone_paths(drones, orders):
    plt.figure(figsize=(10, 10))
    plt.scatter(0, 0, color='black', s=150, label='Warehouse')
    
    colors = ['red', 'green', 'blue', 'magenta', 'cyan']
    
    for i, drone in enumerate(drones):
        if drone.orders:
            path_x, path_y = [0], [0]
            for order_id in drone.orders:
                order = next(o for o in orders if o.id == order_id)
                path = astar_path((path_x[-1], path_y[-1]), order.destination)
                path_x.extend([p[0] for p in path])
                path_y.extend([p[1] for p in path])
                plt.scatter(order.destination[0], order.destination[1], color=colors[i % len(colors)], s=100, label=f"Order {order.id}")
                plt.text(order.destination[0], order.destination[1], f"{order.id}", fontsize=12, ha='right')
            
            return_path = astar_path((path_x[-1], path_y[-1]), (0, 0))
            path_x.extend([p[0] for p in return_path])
            path_y.extend([p[1] for p in return_path])
            plt.plot(path_x, path_y, color=colors[i % len(colors)], label=f"Drone {drone.id} Path")
    
    plt.title("Drone Delivery Routes")
    plt.xlabel("City X-Axis")
    plt.ylabel("City Y-Axis")
    plt.legend()
    plt.grid()
    plt.show()


# Read input JSON data
with open('testcase1.json', 'r') as file:
    data = json.load(file)

# Create drone and order objects from JSON
drones = [
    Drone(
        id=drone["id"],
        speed=drone["speed"],
        capacity=drone["max_payload"],
        max_distance=drone["max_distance"],
        available=drone["available"]
    )
    for drone in data["drones"]["fleet"]
]

orders = [
    Order(
        id=order["id"],
        destination=(order["delivery_x"], order["delivery_y"]),
        weight=order["package_weight"],
        deadline=order["deadline"]
    )
    for order in data["orders"]
]

# Assign orders and visualize paths
assign_orders(drones, orders)

# Print drone assignments with delivery order
for drone in drones:
    if drone.orders:
        print(f"Drone {drone.id} Delivery Order:")
        for index, order_id in enumerate(drone.orders, start=1):
            print(f"  {index}. Order {order_id}")
        print(f"TOTAL DISTANCE: {drone.total_distance}, TRAVEL TIME: {drone.travel_time:.1f} min\n")

# Save assignments to a JSON file
output = {"assignments": [{"drone": drone.id, "orders": drone.orders, "total_distance": drone.total_distance} for drone in drones if drone.orders]}
with open('output.json', 'w') as f:
    json.dump(output, f, indent=4)

print("Drone assignments saved to 'output.json'.")

# visualize paths
plot_drone_paths(drones, orders)



