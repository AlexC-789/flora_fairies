import heapq
def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


def get_neighbors(pos, grid, goal=None):
    row, col = pos
    neighbors = []
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    for dr, dc in directions:
        new_row, new_col = row + dr, col + dc
        if 0 <= new_row < len(grid) and 0 <= new_col < len(grid[0]):
            # Walkable if: free tile (0) OR it's the goal
            if grid[new_row][new_col] == 0 or (goal and (new_row, new_col) == goal):
                neighbors.append((new_row, new_col))
    
    return neighbors


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


def astar(grid, start, goal):
    open_set = []
    closed_set = set()
    came_from = {}
    g_score = {start: 0}

    f_start = heuristic(start, goal)
    heapq.heappush(open_set, (f_start, start))
    
    while open_set:
        f_score, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        closed_set.add(current)

        for neighbor in get_neighbors(current, grid, goal):  # Pass goal here
            if neighbor in closed_set:
                continue
        
            tentative_g = g_score[current] + 1

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))
    
    return None


# Test
if __name__ == "__main__":
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [2, 0, 0, 0, 0],
    ]
    
    start = (4, 0)
    goal = (1, 1)
    
    path = astar(grid, start, goal)
    print(f"Path from {start} to {goal}:")
    print(path)