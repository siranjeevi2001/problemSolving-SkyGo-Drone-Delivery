from update_drone import a_star


goal = (5, 5)

start = (0, 0)


# Grid size and obstacles
grid_size = 20
# obstacles = {(5, 5), (6, 5), (7, 5), (5, 10), (4, 11),(12,8)}
obstacles = {(5, 5), (6, 5), (7, 5),(2,9),(2,10), (2, 11),(12,8)}


data = a_star(start,goal,obstacles,grid_size)

print(data)