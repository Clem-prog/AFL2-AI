import math
import heapq
import collections

graph = {
    'Indonesia': {'German': 200, 'America': 150, 'Singapore': 30},
    'German': {'Indonesia': 200, 'America': 230, 'Italy': 40, 'Brazil': 250},
    'America': {'German': 230, 'Indonesia': 150, 'Singapore': 100, 'Spain': 60, 'Japan': 80, 'Italy': 95, 'Atlantis': 100},
    'Singapore': {'Indonesia': 30, 'America': 100, 'Spain': 75},
    'Spain': {'Singapore': 75, 'America': 60, 'Japan': 90},
    'Japan': {'Spain': 90, 'America': 80, 'Atlantis': 110},
    'Atlantis': {'Japan': 110, 'America': 100, 'Italy': 70, 'Africa': 170},
    'Italy': {'German': 40, 'Brazil': 215, 'Korea': 240, 'Africa': 140, 'Atlantis': 70, 'America': 95},
    'Brazil': {'German': 250, 'Italy': 215, 'Korea': 95},
    'Korea': {'Brazil': 95, 'Italy': 240, 'Africa': 180},
    'Africa': {'Korea': 180, 'Italy': 140, 'Atlantis': 170}
}

#Coordinates for heuristic
positions = {
    'Indonesia': (0, 3),
    'Singapore': (1, 1),
    'Spain': (2, 0),
    'America': (2, 2),
    'German': (2, 4),
    'Italy': (4, 3),
    'Brazil': (4, 5),
    'Korea': (6, 4),
    'Africa': (6, 2),
    'Atlantis': (5, 1),
    'Japan': (4, 0)
}

def generate_heuristic(goal):
    heuristic = {}
    gx, gy = positions[goal]
    for node, (x, y) in positions.items():
        # Euclidean distance to calculate heuristic since we're using coordinates
        heuristic[node] = math.sqrt((x - gx)**2 + (y - gy)**2) * 10
    return heuristic

#A* Search Algorithm
def astar(graph, start, goal, heuristic):
    open_set = []
    heapq.heappush(open_set, (0, start))  # (f_cost, node)
    came_from = {}

    g_cost = {node: math.inf for node in graph}
    g_cost[start] = 0

    f_cost = {node: math.inf for node in graph}
    f_cost[start] = heuristic[start]

    while open_set:
        current_f, current_node = heapq.heappop(open_set)

        if current_f > f_cost[current_node]:
            continue

        # Goal found — reconstruct path
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            return path[::-1], g_cost[goal]

        # Explore neighbors
        for neighbor, cost in graph[current_node].items():
            tentative_g = g_cost[current_node] + cost
            if tentative_g < g_cost[neighbor]:
                came_from[neighbor] = current_node
                g_cost[neighbor] = tentative_g
                f_cost[neighbor] = tentative_g + heuristic[neighbor]
                heapq.heappush(open_set, (f_cost[neighbor], neighbor))

    return None, math.inf

#BFS Seacrh Algorithm

def bfs(graph, start, goal):
    queue = collections.deque([start])
    
    came_from = {}
    visited = {start}
    
    while queue:
        current_node = queue.popleft()

        if current_node == goal:
            path = []
            total_cost = 0
            temp_node = goal

            while temp_node in came_from:
                path.append(temp_node)
                parent_node = came_from[temp_node]
                total_cost += graph[parent_node][temp_node]
                temp_node = parent_node
            path.append(start)
            return path[::-1], total_cost

        # Explore neighbors
        for neighbor, cost in graph[current_node].items():
           
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current_node
                queue.append(neighbor)

    return None, math.inf

# BAGIAN INI TERGANTUNG MAU DIPAKAI DI GUI ATAU NGGAK, KLO NGGAK HAPUS AJA YEAH :D, YANG PENTING OUTPUTNYA JANGAN DARI TERMINAL
def main():
    start = "Indonesia"
    goal = "Atlantis"
    heuristic = generate_heuristic("Atlantis")
    path, cost = astar(graph, start, goal, heuristic)

    if path:
        print(f"\n🛫 Shortest path from {start} to {goal}: {' → '.join(path)}")
        print(f"💰 Total travel cost: {cost}")
    else:
        print("No path found between these airports.")


    print("--- Running BFS Search  ---")
    bfs_path, bfs_cost = bfs(graph, start, goal) 

    if bfs_path:
        print(f"🛫 BFS Path from {start} to {goal}: {' → '.join(bfs_path)}")
        print(f"💰 Total travel cost: {bfs_cost}")
    else:
        print("No path found between these airports.")

main()