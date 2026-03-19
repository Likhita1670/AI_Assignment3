import heapq, random, math, time

SIZE = 70
DENSITY = {'low': 0.1, 'medium': 0.25, 'high': 0.4}

def make_grid(density, start, goal):
    grid = [[1 if random.random() < density else 0 for _ in range(SIZE)] for _ in range(SIZE)]
    grid[start[0]][start[1]] = 0
    grid[goal[0]][goal[1]] = 0
    return grid

def astar(grid, start, goal):
    h = lambda a: math.dist(a, goal)
    open_set = [(h(start), 0, start, [start])]
    visited = set()
    while open_set:
        _, cost, cur, path = heapq.heappop(open_set)
        if cur in visited: continue
        visited.add(cur)
        if cur == goal:
            return path, len(visited), cost
        r, c = cur
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < SIZE and 0 <= nc < SIZE and grid[nr][nc] == 0 and (nr,nc) not in visited:
                step = math.sqrt(2) if dr and dc else 1
                heapq.heappush(open_set, (cost+step+h((nr,nc)), cost+step, (nr,nc), path+[(nr,nc)]))
    return None, len(visited), 0

def draw(grid, path, start, goal):
    path_set = set(path)
    print()
    for r in range(SIZE):
        row = ""
        for c in range(SIZE):
            if (r,c) == start:      row += "S"
            elif (r,c) == goal:     row += "G"
            elif (r,c) in path_set: row += "."
            elif grid[r][c]:        row += "#"
            else:                   row += " "
        print(row)

#  Input 
print(f"Grid: {SIZE}x{SIZE}  |  Coords: 0 to {SIZE-1}")
sr, sc = map(int, input("Start (row col): ").split())
gr, gc = map(int, input("Goal  (row col): ").split())
level  = input("Density (low/medium/high): ").strip().lower()

start, goal = (sr, sc), (gr, gc)
grid = make_grid(DENSITY[level], start, goal)

t0 = time.time()
path, explored, dist = astar(grid, start, goal)
elapsed = time.time() - t0

if not path:
    print("No path found!")
else:
    draw(grid, path, start, goal)
    print(f"\n--- Results ---")
    print(f"Path length   : {len(path)} steps")
    print(f"Distance      : {dist:.2f} km")
    print(f"Direct dist   : {math.dist(start, goal):.2f} km")
    print(f"Nodes explored: {explored}")
    print(f"Time taken    : {elapsed*1000:.2f} ms")
