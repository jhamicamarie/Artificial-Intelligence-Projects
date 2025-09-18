import heapq
import matplotlib.pyplot as plt
import numpy as np

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start, [start]))
    visited = set()
    all_visited = []

    while open_list:
        f, g, current, path = heapq.heappop(open_list)

        if current == goal:
            return path, all_visited

        if current in visited:
            continue
        visited.add(current)
        all_visited.append(current)

        x, y = current
        for dx, dy in [(0,1),(1,0),(0,-1),(-1,0)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                if (nx, ny) not in visited:
                    new_g = g + 1
                    new_f = new_g + heuristic((nx, ny), goal)
                    heapq.heappush(open_list, (new_f, new_g, (nx, ny), path + [(nx, ny)]))
    return None, all_visited

def visualize(grid, path, visited, start, goal):
    rows, cols = len(grid), len(grid[0])
    maze_img = np.ones((rows, cols, 3))

    for x in range(rows):
        for y in range(cols):
            if grid[x][y] == 1:
                plt.scatter(y, x, marker='s', color='black', s=400)

    if visited:
        vx, vy = zip(*visited)
        plt.scatter(vy, vx, color="#77DD77", s=80, label="Visited")

    if path:
        px, py = zip(*path)
        plt.plot(py, px, marker='o', color="#4B5320", linewidth=3, markersize=8, label="Path")

    plt.scatter(start[1], start[0], marker='o', color='green', s=200, label="Start")
    plt.scatter(goal[1], goal[0], marker='X', color='blue', s=200, label="Goal")

    plt.xticks([])
    plt.yticks([])
    plt.gca().invert_yaxis()
    plt.gca().set_facecolor("white")

    plt.legend()
    plt.title("A* Maze Solver by Antonio and Mundos")
    plt.show()

grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0]
]
start = (0, 0)
goal = (4, 4)

path, visited = astar(grid, start, goal)
print("Path found:", path)

visualize(grid, path, visited, start, goal)