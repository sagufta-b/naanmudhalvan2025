import heapq

# Define the grid size
grid_size = 10
grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# Define obstacles (represented by 1)
obstacles = [(3, 3), (3, 4), (3, 5), (5, 5), (6, 5)]
for x, y in obstacles:
    grid[y][x] = 1

# Define start and goal positions
start = (0, 0)
goal = (9, 9)

# Heuristic function for A* (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* pathfinding algorithm
def astar(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        neighbors = [
            (current[0] + dx, current[1] + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]

        for neighbor in neighbors:
            x, y = neighbor
            if 0 <= x < grid_size and 0 <= y < grid_size:
                if grid[y][x] == 1:
                    continue  # Skip obstacles

                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

    return None  # No path found

# Find path
path = astar(start, goal)

# Display the grid and path
for y in range(grid_size):
    row = ''
    for x in range(grid_size):
        if (x, y) == start:
            row += 'S '
        elif (x, y) == goal:
            row += 'G '
        elif (x, y) in obstacles:
            row += 'X '
        elif path and (x, y) in path:
            row += '* '
        else:
            row += '. '
    print(row)
