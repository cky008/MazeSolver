import gui as gui
from pyamaze import maze, agent, textLabel, COLOR
import algorithms as algorithms
import matplotlib.pyplot as plt

global m, size_of_maze, goal_of_maze


def get_maze_agent_path():
    algo_list = gui.algo_choices
    bfs_path = None
    dfs_path = None
    a_star_path = None
    val_path = None
    pol_path = None
    direction = gui.direction_of_search.get()
    algorithms.NODES = direction
    print(f"Direction of Search: {direction}")
    agents = [None, None, None, None, None]  # 0: BFS, 1: DFS, 2: A*, 3: MDP Value, 4: MDP Policy
    explored_agents = [None, None, None]  # 0: BFS, 1: DFS, 2: A*
    explored_paths_list = [None, None, None]  # 0: BFS, 1: DFS, 2: A*
    path_dic = {}
    path_lengths = {}
    calculated_times = {}
    memory_consumptions = {}
    visited_counts = {}

    def plot_comparisons():
        fig, axs = plt.subplots(1, 3, figsize=(14, 6))

        # Path Length Comparison
        bars = axs[0].bar(path_lengths.keys(), path_lengths.values(), color='skyblue')
        axs[0].set_xlabel('Algorithm')
        axs[0].set_ylabel('Path Length')
        axs[0].set_title(f'Path Length Comparison for {m.rows}x{m.cols} Maze')
        for bar in bars:
            yval = bar.get_height()
            axs[0].text(bar.get_x() + bar.get_width() / 2.0, yval, round(yval, 2),
                        va='bottom')  # va: vertical alignment

        # Time Comparison
        bars = axs[1].bar(calculated_times.keys(), calculated_times.values(), color='lightgreen')
        axs[1].set_xlabel('Algorithm')
        axs[1].set_ylabel('Calculated Time (ms)')
        axs[1].set_title(f'Calculated Time Comparison for {m.rows}x{m.cols} Maze')
        for bar in bars:
            yval = bar.get_height()
            axs[1].text(bar.get_x() + bar.get_width() / 2.0, yval, round(yval, 2), va='bottom')

        # Memory Comparison
        bars = axs[2].bar(memory_consumptions.keys(), memory_consumptions.values(), color='salmon')
        axs[2].set_xlabel('Algorithm')
        axs[2].set_ylabel('Memory Consumption (MB)')
        axs[2].set_title(f'Memory Consumption Comparison for {m.rows}x{m.cols} Maze')
        for bar in bars:
            yval = bar.get_height()
            axs[2].text(bar.get_x() + bar.get_width() / 2.0, yval, round(yval, 2), va='bottom')

        plt.tight_layout()
        plt.savefig(f'images/Path-Time-Memory-{m.rows}-{m.cols}.eps', format='eps')
        plt.show()
        print(f"For {m.rows}x{m.cols} Maze:"
              f"Path Lengths: {path_lengths}"
              f"\nCalculated Times: {calculated_times}"
              f"\nMemory Consumptions: {memory_consumptions}"
              f"\nVisited Counts: {visited_counts}")

        # For searching algorithms: Visited Nodes Comparison
        if visited_counts:
            plt.figure(figsize=(10, 7))
            bars = plt.bar(visited_counts.keys(), visited_counts.values(), color='gold')
            plt.xlabel('Algorithm')
            plt.ylabel('Visited Nodes Count')
            plt.title(f'Visited Nodes Comparison for {m.rows}x{m.cols} Maze')
            for bar in bars:
                yval = bar.get_height()
                plt.text(bar.get_x() + bar.get_width() / 2.0, yval, round(yval, 2), va='bottom')

            plt.savefig(f'images/Visited-{m.rows}-{m.cols}.eps')
            plt.show()

    print(f"Algorithms: {', '.join(algo_list)}")

    for algo in algo_list:
        print(f"Calculating path for {algo}...")
        if algo == 'BFS':
            bfs_instance = algorithms.BFS(m=m, goal=goal_of_maze)
            bfs_instance.nodes = direction
            bfs_instance.execute_algorithm()
            bfs_path, visited_counts['BFS'], bfs_calculated_time, bfs_memory_consumption = bfs_instance.get_final_path()
            b = agent(parentMaze=m,
                      x=None, y=None,
                      shape='square', footprints=True, filled=True,
                      color=COLOR.yellow)
            agents[0] = b
            be = agent(parentMaze=m,
                       x=None, y=None,
                       shape='arrow', footprints=True, filled=True,
                       color=COLOR.yellow)
            explored_agents[0] = be
            explored_paths_list[0] = bfs_instance.search_path
            path_lengths['BFS'] = len(bfs_path)
            calculated_times['BFS'] = round(bfs_calculated_time * 1000, 4)
            memory_consumptions['BFS'] = round((bfs_memory_consumption / (1024 * 1024)), 3)
            textLabel(m, f'BFS Path', path_lengths['BFS'])
            textLabel(m, f'BFS V', visited_counts['BFS'])
            textLabel(m, f'BFS T', calculated_times['BFS'])
            textLabel(m, f'BFS MEM', memory_consumptions['BFS'])
        if algo == 'DFS':
            dfs_instance = algorithms.DFS(m=m, goal=goal_of_maze)
            dfs_instance.nodes = direction
            dfs_instance.execute_algorithm()
            dfs_path, visited_counts['DFS'], dfs_calculated_time, dfs_memory_consumption = dfs_instance.get_final_path()
            d = agent(parentMaze=m,
                      x=None, y=None,
                      shape='square', footprints=True, filled=True,
                      color=COLOR.cyan)
            agents[1] = d
            de = agent(parentMaze=m,
                       x=None, y=None,
                       shape='arrow', footprints=True, filled=True,
                       color=COLOR.cyan)
            explored_agents[1] = de
            explored_paths_list[1] = dfs_instance.search_path
            path_lengths['DFS'] = len(dfs_path)
            calculated_times['DFS'] = round(dfs_calculated_time * 1000, 4)
            memory_consumptions['DFS'] = round((dfs_memory_consumption / (1024 * 1024)), 3)
            textLabel(m, f'DFS Path', path_lengths['DFS'])
            textLabel(m, f'DFS V', visited_counts['DFS'])
            textLabel(m, f'DFS T', calculated_times['DFS'])
            textLabel(m, f'DFS MEM', memory_consumptions['DFS'])
        if algo == 'A*':
            a_star_instance = algorithms.AStar(m=m, goal=goal_of_maze)
            if a_star_instance.manhattan_flag:
                print("Using manhattan distance for A*..")
            else:
                print("Using euclidean distance for A*..")
            a_star_instance.nodes = direction
            a_star_instance.execute_algorithm()
            a_star_path, visited_counts[
                'A*'], a_star_calculated_time, a_star_memory_consumption = a_star_instance.get_final_path()
            a = agent(parentMaze=m,
                      x=None, y=None,
                      shape='square', footprints=True, filled=False,
                      color=COLOR.green)
            agents[2] = a
            ae = agent(parentMaze=m,
                       x=None, y=None,
                       shape='arrow', footprints=True, filled=False,
                       color=COLOR.green)
            explored_agents[2] = ae
            explored_paths_list[2] = a_star_instance.search_path
            path_lengths['A*'] = len(a_star_path)
            calculated_times['A*'] = round(a_star_calculated_time * 1000, 4)
            memory_consumptions['A*'] = round((a_star_memory_consumption / (1024 * 1024)), 3)
            textLabel(m, f'A* Path', path_lengths['A*'])
            textLabel(m, f'A* V', visited_counts['A*'])
            textLabel(m, f'A* T', calculated_times['A*'])
            textLabel(m, f'A* MEM', memory_consumptions['A*'])
        if algo == 'VAL':
            value_iteration_instance = algorithms.ValueIteration(m=m, goal=goal_of_maze)
            value_iteration_instance.execute_iteration()
            value_iteration_instance.plot_maze_weights()
            val_path, val_calculated_time, val_memory_consumption = value_iteration_instance.get_final_path()
            v = agent(parentMaze=m,
                      x=None, y=None,
                      shape='square', footprints=True, filled=True,
                      color=COLOR.blue)
            agents[3] = v
            path_lengths['VAL'] = len(val_path) + 1
            calculated_times['VAL'] = round(val_calculated_time * 1000, 4)
            memory_consumptions['VAL'] = round((val_memory_consumption / (1024 * 1024)), 3)
            textLabel(m, f'VI Path', len(val_path) + 1)
            textLabel(m, f'VI T', calculated_times['VAL'])
            textLabel(m, f'VI MEM', memory_consumptions['VAL'])

        if algo == 'POL':
            policy_iteration_instance = algorithms.PolicyIteration(m=m, goal=goal_of_maze)
            policy_iteration_instance.execute_iteration()
            policy_iteration_instance.plot_maze_weights()
            pol_path, pol_calculated_time, pol_memory_consumption = policy_iteration_instance.get_final_path()

            p = agent(parentMaze=m,
                      x=None, y=None,
                      shape='square', footprints=True, filled=False,
                      color=COLOR.red)
            agents[4] = p
            path_lengths['POL'] = len(pol_path) + 1
            calculated_times['POL'] = round(pol_calculated_time * 1000, 4)
            memory_consumptions['POL'] = round((pol_memory_consumption / (1024 * 1024)), 3)
            textLabel(m, f'PI Path', path_lengths['POL'])
            textLabel(m, f'PI T', calculated_times['POL'])
            textLabel(m, f'PI MEM', memory_consumptions['POL'])

    for index, value in enumerate([bfs_path, dfs_path, a_star_path, val_path, pol_path]):
        if value is not None:
            path_dic[agents[index]] = value

    explored_paths_dic = create_dict(explored_agents, explored_paths_list)

    plot_comparisons()

    return [path_dic, explored_paths_dic]


def create_dict(agents, paths):
    return {agent: path for agent, path in zip(agents, paths) if path is not None}


def main():
    global m, size_of_maze, goal_of_maze
    gui.window.mainloop()
    print("Running GUI")
    if gui.run_flag:
        maze_var = gui.maze_var
        rows, cols = size_of_maze = (int(gui.rows), int(gui.cols))
        goal_x, goal_y = goal_of_maze = (int(gui.goal_x), int(gui.goal_y))
        if maze_var == "custom":
            m = maze(rows, cols)
            print(f"Custom Maze: Rows={rows}, Cols={cols}, Goal=({goal_x}, {goal_y})")
            print("Creating Maze...")
            m.CreateMaze(x=goal_x, y=goal_y, pattern=None, theme=COLOR.light, loopPercent=100)
            # m.CreateMaze(x=goal_x, y=goal_y, pattern=None, theme=COLOR.light, loopPercent=100, saveMaze=True)
        else:
            maze_var_arr = maze_var.split("-")
            rows, cols = size_of_maze = (int(maze_var_arr[0]), int(maze_var_arr[1]))
            m = maze(rows, cols)
            print(f"Predefined Maze: {maze_var}, Rows={rows}, Cols={cols} Goal=({goal_x}, {goal_y})")
            print("Creating Maze...")
            m.CreateMaze(x=goal_x, y=goal_y, pattern=None, theme=COLOR.light, loopPercent=100,
                         loadMaze=f'./savedMaze/maze--{gui.maze_var}.csv')
        print("Running Maze with path animations...")
        maze_path = get_maze_agent_path()
        if maze_path:
            if gui.process_flag:
                print("Show Explored Path Selected.")
                print("Drawing explored paths...")
                m.tracePath(maze_path[1], delay=3, showMarked=True, kill=True)
            print("Drawing final paths...")
            m.tracePath(maze_path[0], delay=3, showMarked=True)
            m.run()
        else:
            m.run()
            raise AssertionError("No path data available.")


if __name__ == '__main__':
    main()
