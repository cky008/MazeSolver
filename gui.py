from itertools import permutations
from tkinter import *
import os

rows, cols, goal_x, goal_y = 50, 50, 1, 1
mdp_det = False
algo_choices = []
run_flag = False
process_flag = False


def run():
    # Debugging
    global maze_var
    maze_var = maze_var.get()
    if maze_var == "custom":
        global rows, cols, goal_x, goal_y
        rows, cols = rows_var.get(), cols_var.get()
        print(f"GUI: Rows={rows}, Cols={cols}, Goal=({goal_x}, {goal_y})")
    else:
        print(f"GUI Predefined Maze: {maze_var}")
    goal_x, goal_y = goal_x_var.get(), goal_y_var.get()
    global algo_choices, mdp_det, process_flag
    algo_choices = []
    if bfs_var.get():
        algo_choices.append('BFS')
    if dfs_var.get():
        algo_choices.append('DFS')
    if astar_var.get():
        algo_choices.append('A*')
    if val_var.get():
        algo_choices.append('VAL')
    if pol_var.get():
        algo_choices.append('POL')
    process_flag = process_var.get()
    global run_flag
    run_flag = True
    print("Running...", run_flag)
    window.after(100, window.destroy)


window = Tk()
window.title('Maze Solver')
window.geometry("500x500")
window.config(pady=5)

# ---Given Maze---
maze_frame = Frame(window)
maze_frame.pack(fill="both", expand=True, padx=10, pady=10)
maze_var = StringVar(value="custom")
maze_options = ["custom"]  # default option
# get all maze files
maze_files = os.listdir('./savedMaze')


# extract the size from the filename and add it to the maze_options
def extract_size(filename):
    parts = filename.replace('maze--', '').replace('.csv', '').split('-')
    if len(parts) == 2:
        return tuple(map(int, parts))
    return 0, 0


sorted_files = sorted(maze_files, key=extract_size)

for filename in sorted_files:
    if '--' in filename:
        size = filename.split('--')[1].replace('.csv', '')
        maze_options.append(size)
    else:
        # 跳过这个文件，或者给它一个默认的大小
        continue


for option in maze_options:
    Radiobutton(maze_frame, text=option, variable=maze_var, value=option).pack(anchor="w")

# ---Custom Maze---
custom_frame = Frame(window)
custom_frame.pack(fill="both", expand=True, padx=10, pady=10)

Label(custom_frame, text="Rows:").grid(row=0, column=0)
rows_var = StringVar(value="50")
Entry(custom_frame, textvariable=rows_var).grid(row=1, column=0)

Label(custom_frame, text="Cols:").grid(row=0, column=1)
cols_var = StringVar(value="50")
Entry(custom_frame, textvariable=cols_var).grid(row=1, column=1)

Label(custom_frame, text="Goal X:").grid(row=2, column=0)
goal_x_var = StringVar(value="1")
Entry(custom_frame, textvariable=goal_x_var).grid(row=3, column=0)

Label(custom_frame, text="Goal Y:").grid(row=2, column=1)
goal_y_var = StringVar(value="1")
Entry(custom_frame, textvariable=goal_y_var).grid(row=3, column=1)

Label(custom_frame, text="Direction of Search (Default: NWSE)").grid(row=4, column=0)
direction_of_search = StringVar(value="NWSE")
options_of_direction_of_search = [''.join(p) for p in permutations('NWSE')]
OptionMenu(custom_frame, direction_of_search, *options_of_direction_of_search).grid(row=4, column=1)

# ---Algorithms---
algo_frame = Frame(window)
algo_frame.pack(fill="both", expand=True, padx=10, pady=10, side="left")
bfs_var = IntVar()
dfs_var = IntVar()
astar_var = IntVar()
Checkbutton(algo_frame, text="BFS", variable=bfs_var).pack(anchor="w")
Checkbutton(algo_frame, text="DFS", variable=dfs_var).pack(anchor="w")
Checkbutton(algo_frame, text="A*", variable=astar_var).pack(anchor="w")

# ---MDP---
mdp_frame = Frame(window)
mdp_frame.pack(fill="both", expand=True, padx=10, pady=10, side="left")
val_var = IntVar(value=0)
pol_var = IntVar(value=0)
Checkbutton(mdp_frame, text="Value Iteration", variable=val_var).pack(anchor="w")
Checkbutton(mdp_frame, text="Policy Iteration", variable=pol_var).pack(anchor="w")

# ---Run Button---
run_frame = Frame(window)
run_frame.pack(fill="both", expand=True, padx=10, pady=10)
Button(run_frame, text="Run", command=run).pack()
process_var = IntVar(value=0)
Checkbutton(run_frame, text="Show Process", variable=process_var).pack()
