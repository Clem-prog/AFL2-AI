import math
import heapq
import collections
import tkinter as tk
from tkinter import ttk, messagebox


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
    nodes_explored = 0

    g_cost = {node: math.inf for node in graph}
    g_cost[start] = 0

    f_cost = {node: math.inf for node in graph}
    f_cost[start] = heuristic[start]

    while open_set:
        current_f, current_node = heapq.heappop(open_set)
        nodes_explored += 1

        if current_f > f_cost[current_node]:
            continue

        # Goal found — reconstruct path
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            return path[::-1], g_cost[goal], nodes_explored

        # Explore neighbors
        for neighbor, cost in graph[current_node].items():
            tentative_g = g_cost[current_node] + cost
            if tentative_g < g_cost[neighbor]:
                came_from[neighbor] = current_node
                g_cost[neighbor] = tentative_g
                f_cost[neighbor] = tentative_g + heuristic[neighbor]
                heapq.heappush(open_set, (f_cost[neighbor], neighbor))

    return None, math.inf, nodes_explored

#BFS Seacrh Algorithm

def bfs(graph, start, goal):
    queue = collections.deque([start])
    
    came_from = {}
    visited = {start}
    nodes_explored = 0
    
    while queue:
        current_node = queue.popleft()
        nodes_explored += 1
    
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
            return path[::-1], total_cost, nodes_explored

        # Explore neighbors
        for neighbor, cost in graph[current_node].items():
           
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current_node
                queue.append(neighbor)

    return None, math.inf, nodes_explored

#GUI
def run_comparison():
    start = start_var.get()
    goal = goal_var.get()

    if start == goal:
        messagebox.showwarning("Warning", "Start and goal cannot be the same!")
        return

    heuristic = generate_heuristic(goal)

    #Run A* and BFS
    a_path, a_cost, a_nodes = astar(graph, start, goal, heuristic)
    b_path, b_cost, b_nodes = bfs(graph, start, goal)

    output_text.delete(1.0, tk.END)

    #Display & compare results
    if a_path:
        output_text.insert(tk.END, f"🌟 A* Search:\n")
        output_text.insert(tk.END, f"  Path: {' → '.join(a_path)}\n")
        output_text.insert(tk.END, f"  Cost: {a_cost}\n")
        output_text.insert(tk.END, f"  Nodes Explored: {a_nodes}\n\n")
    else:
        output_text.insert(tk.END, "❌ A* could not find a path.\n\n")

    if b_path:
        output_text.insert(tk.END, f"🔵 BFS Search:\n")
        output_text.insert(tk.END, f"  Path: {' → '.join(b_path)}\n")
        output_text.insert(tk.END, f"  Cost: {b_cost}\n")
        output_text.insert(tk.END, f"  Nodes Explored: {b_nodes}\n\n")
    else:
        output_text.insert(tk.END, "❌ BFS could not find a path.\n\n")

    if a_path and b_path:
        cheaper = "A*" if a_cost < b_cost else "BFS" if b_cost < a_cost else "Both are equal"
        result_label.config(
            text=f"🏆 Cheaper path: {cheaper} | A*: {a_cost} | BFS: {b_cost}",
            fg="#008000" if cheaper != "Both are equal" else "#FF9800"
        )
    else:
        result_label.config(text="❌ No valid comparison (one or both failed)", fg="#d32f2f")


def clear_output():
    output_text.delete(1.0, tk.END)
    result_label.config(text="")
    start_var.set("Indonesia")
    goal_var.set("Atlantis")


# Main Window
root = tk.Tk()
root.geometry("850x550")
root.title("✈️ Plane Route Finder | Compare A* & BFS")
root.configure(bg="#F5F5F5")
root.minsize(850, 550)

#Title
title_label = tk.Label(root, text="🌍 Plane Route Finder (Compare A* & BFS)", font=("Arial", 22, "bold"), bg="#F5F5F5", fg="#333")
title_label.pack(pady=10)

#Dropdown
frame = tk.Frame(root, bg="#F5F5F5")
frame.pack(pady=10)

tk.Label(frame, text="Start:", font=("Arial", 14), bg="#F5F5F5").grid(row=0, column=0, padx=10)
tk.Label(frame, text="Goal:", font=("Arial", 14), bg="#F5F5F5").grid(row=0, column=2, padx=10)

start_var = tk.StringVar(value="Indonesia")
goal_var = tk.StringVar(value="Atlantis")
airport_list = list(graph.keys())

start_menu = ttk.Combobox(frame, textvariable=start_var, values=airport_list, font=("Arial", 12), state="readonly", width=15)
goal_menu = ttk.Combobox(frame, textvariable=goal_var, values=airport_list, font=("Arial", 12), state="readonly", width=15)
start_menu.grid(row=0, column=1)
goal_menu.grid(row=0, column=3)

#Buttons
btn_frame = tk.Frame(root, bg="#F5F5F5")
btn_frame.pack(pady=15)

compare_btn = tk.Button(btn_frame, text="Compare A* and BFS", font=("Arial", 14), bg="#673AB7", fg="white", padx=20, command=run_comparison)
clear_btn = tk.Button(btn_frame, text="Clear Output", font=("Arial", 14), bg="#f44336", fg="white", padx=20, command=clear_output)

compare_btn.grid(row=0, column=0, padx=10)
clear_btn.grid(row=0, column=1, padx=10)

#Output
output_frame = tk.Frame(root)
output_frame.pack(padx=20, pady=10, fill=tk.BOTH, expand=True)

output_text = tk.Text(output_frame, height=12, font=("Arial", 14), wrap="word")
output_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

result_label = tk.Label(root, text="", font=("Arial", 15, "bold"), bg="#F5F5F5", fg="#222")
result_label.pack(pady=10)

root.mainloop()

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