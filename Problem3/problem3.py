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

# Input 
print(f"Grid: {SIZE}x{SIZE}  |  Coords: 0 to {SIZE-1}")
sr, sc = map(int, input("Start (row col): ").split())
gr, gc = map(int, input("Goal  (row col): ").split())
level  = input("Density (low/medium/high): ").strip().lower()

start, goal = (sr, sc), (gr, gc)
grid = make_grid(DENSITY[level], start, goal)

# Dynamic A* 
print("\n=== DYNAMIC OBSTACLES ===")
sensor = int(input("Sensor range (e.g. 5): "))

known = [[0]*SIZE for _ in range(SIZE)]
pos, dyn_path, plan, replans = start, [start], [], 0

t0 = time.time()
while pos != goal:
    for dr in range(-sensor, sensor+1):
        for dc in range(-sensor, sensor+1):
            nr, nc = pos[0]+dr, pos[1]+dc
            if 0 <= nr < SIZE and 0 <= nc < SIZE:
                if grid[nr][nc] == 1 and known[nr][nc] == 0:
                    known[nr][nc] = 1
                    plan = []

    if not plan:
        plan, _, _ = astar(known, pos, goal)
        replans += 1
        if not plan: print("Blocked!"); break
        plan = plan[1:]

    pos = plan.pop(0)
    dyn_path.append(pos)

elapsed = time.time() - t0

draw(grid, dyn_path, start, goal)
dyn_dist = sum(math.dist(dyn_path[i], dyn_path[i+1]) for i in range(len(dyn_path)-1))
print(f"Distance      : {dyn_dist:.2f} km")
print(f"Replans made  : {replans}")
print(f"Time taken    : {elapsed*1000:.2f} ms")
