import csv
import heapq
from collections import defaultdict

# Load graph
def load_graph(filename):
    graph = defaultdict(list)

    with open(filename, 'r') as file:
        reader = csv.DictReader(file)

        for row in reader:
            src = row['source']
            dest = row['destination']
            dist = int(row['distance'])

            graph[src].append((dest, dist))
            graph[dest].append((src, dist))  # undirected

    return graph


# Dijkstra with path tracking
def dijkstra(graph, start, goal):
    pq = [(0, start)]
    distances = {node: float('inf') for node in graph}
    previous = {node: None for node in graph}

    distances[start] = 0

    while pq:
        current_cost, current_node = heapq.heappop(pq)

        if current_node == goal:
            break

        for neighbor, weight in graph[current_node]:
            new_cost = current_cost + weight

            if new_cost < distances[neighbor]:
                distances[neighbor] = new_cost
                previous[neighbor] = current_node
                heapq.heappush(pq, (new_cost, neighbor))

    return distances, previous


# Reconstruct path
def get_path(previous, start, goal):
    path = []
    current = goal

    while current:
        path.append(current)
        current = previous[current]

    path.reverse()

    if path[0] == start:
        return path
    return None


# Main
if __name__ == "__main__":
    graph = load_graph("data.csv")

    print("Available cities:")
    for city in graph:
        print(city)

    start = input("\nEnter start city: ").strip()
    goal = input("Enter goal city: ").strip()

    if start not in graph or goal not in graph:
        print("Invalid city name")
    else:
        distances, previous = dijkstra(graph, start, goal)
        path = get_path(previous, start, goal)

        if distances[goal] == float('inf'):
            print("\nNo path found")
        else:
            print(f"\nShortest distance: {distances[goal]} km")
            print("Path:", " -> ".join(path))
