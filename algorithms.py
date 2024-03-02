from queue import Queue, PriorityQueue
import time
import tracemalloc as memory_trace
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt


def start_memory_tracing():
    memory_trace.stop()
    memory_trace.start()


class GraphSearchAlgorithms:
    def __init__(self, m, goal):
        self.m = m
        self.cost_time = 0
        self.memory_peak = 0
        self.goal = goal
        self.start_node = (self.m.rows, self.m.cols)
        self.visited = set()
        self.nodes = 'NWSE'
        self.final_path = []
        self.search_path = []
        self.origin_dict = {}

    def stop_memory_tracing(self):
        _, self.memory_peak = memory_trace.get_traced_memory()
        return self.memory_peak

    def validate_node(self, current_node):
        if current_node[0] < 1 or current_node[0] > self.m.rows or current_node[1] < 1 or current_node[1] > self.m.cols:
            return False
        if current_node in self.visited:
            return False
        return True

    def compute_next_node(self, current_node):
        for direction in self.nodes:
            if self.m.maze_map[current_node][direction]:
                next_node = None
                if direction == 'N':
                    next_node = (current_node[0] - 1, current_node[1])
                elif direction == 'W':
                    next_node = (current_node[0], current_node[1] - 1)
                elif direction == 'S':
                    next_node = (current_node[0] + 1, current_node[1])
                elif direction == 'E':
                    next_node = (current_node[0], current_node[1] + 1)
                if self.validate_node(next_node):
                    return next_node

    def traceback_final_path(self):
        final_path = []
        current_node = self.goal
        while current_node != self.start_node:
            final_path.append(current_node)
            current_node = self.origin_dict[current_node]
        final_path.append(self.start_node)
        return final_path[::-1]

    def execute_algorithm(self):
        pass

    def get_final_path(self):
        return self.final_path, len(self.search_path), self.cost_time, self.memory_peak


class BFS(GraphSearchAlgorithms):
    def __init__(self, m, goal):
        super().__init__(m, goal)
        self.queue = Queue()
        self.queue.put(self.start_node)

    def execute_algorithm(self):
        start = time.time()
        start_memory_tracing()
        while self.queue:
            current_node = self.queue.get()
            self.search_path.append(current_node)
            if current_node == self.goal:
                self.final_path = self.traceback_final_path()
                end = time.time()
                self.stop_memory_tracing()
                self.cost_time = end - start
                return self.search_path, self.final_path
            while True:
                new_position = self.compute_next_node(current_node)
                if new_position:
                    self.queue.put(new_position)
                    self.visited.add(new_position)
                    self.origin_dict[new_position] = current_node
                else:
                    break
        return self.search_path, None


class DFS(GraphSearchAlgorithms):
    def __init__(self, m, goal):
        super().__init__(m, goal)
        self.final_path = [self.start_node]
        self.search_path = [self.start_node]

    def execute_algorithm(self):
        start = time.time()
        start_memory_tracing()
        while self.final_path:
            current_position = self.final_path.pop()
            self.search_path.append(current_position)
            if current_position == self.goal:
                end = time.time()
                self.stop_memory_tracing()
                self.cost_time = end - start
                self.final_path.append(current_position)
                return self.search_path, self.final_path
            self.visited.add(current_position)
            new_position = self.compute_next_node(current_position)
            if new_position:
                self.final_path.append(current_position)
                self.final_path.append(new_position)
                self.search_path.append(new_position)
        return self.search_path, None


class AStar(GraphSearchAlgorithms):
    def __init__(self, m, goal):
        super().__init__(m, goal)
        self.g_score = {self.start_node: 0}
        self.priority_queue = PriorityQueue()
        self.manhattan_flag = True

    def h_score(self, node, manhattan=True):
        if manhattan:
            return abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])
        else:  # Euclidean
            return sqrt(((node[0] - self.goal[0]) ** 2) + ((node[1] - self.goal[1]) ** 2))

    def execute_algorithm(self):
        start = time.time()
        start_memory_tracing()
        self.priority_queue.put((self.g_score[self.start_node] + self.h_score(self.start_node), self.start_node))

        while self.priority_queue:
            _, current_position = self.priority_queue.get()
            self.search_path.append(current_position)
            if current_position == self.goal:
                end = time.time()
                self.stop_memory_tracing()
                self.cost_time = end - start
                self.final_path = self.traceback_final_path()
                return self.search_path, self.final_path
            self.visited.add(current_position)
            while True:
                new_position = self.compute_next_node(current_position)
                if new_position:
                    self.g_score[new_position] = self.g_score[current_position] + 1
                    self.priority_queue.put((self.g_score[new_position] + self.h_score(new_position), new_position))
                    self.origin_dict[new_position] = current_position
                    self.visited.add(new_position)
                else:
                    break
        return self.search_path, None


def get_direction(current_node):
    return max(current_node, key=current_node.get)


class MarkovDecisionProcess:
    def __init__(self, m=None, goal=None):
        self.m = m
        self.cost_time = 0
        self.memory_peak = 0
        self.final_path = {}
        self.m = m
        if goal is None:
            raise AssertionError("Goal Cannot Be None")
        self.goal = goal
        self.start_node = (self.m.rows, self.m.cols)

        self.discount_factor = 0.9
        self.convergence_threshold = 0.000001
        self.transition_probability = {'N': 1, 'S': 1, 'W': 1, 'E': 1}
        self.transition_dictionary = {key: {subkey: 0 for subkey in self.m.maze_map[key]} for key in self.m.maze_map}

    def start_memory_tracing(self):
        memory_trace.stop()
        memory_trace.start()

    def stop_memory_tracing(self):
        _, self.memory_peak = memory_trace.get_traced_memory()
        return self.memory_peak

    def get_final_path(self):
        next_node = [self.start_node]

        while len(next_node) > 0:
            current_node = next_node.pop()
            if current_node == self.goal:
                break

            direction = self.get_direction_for_current_node(current_node)

            if direction == 'N':
                _temp_next_node_ = (current_node[0] - 1, current_node[1])
            elif direction == 'S':
                _temp_next_node_ = (current_node[0] + 1, current_node[1])
            elif direction == 'W':
                _temp_next_node_ = (current_node[0], current_node[1] - 1)
            elif direction == 'E':
                _temp_next_node_ = (current_node[0], current_node[1] + 1)
            else:
                raise AssertionError("Invalid Direction")
            self.final_path[current_node] = _temp_next_node_
            next_node.append(_temp_next_node_)

        return self.final_path, self.cost_time, self.memory_peak

    def get_direction_for_current_node(self, current_node):
        pass

    def execute_iteration(self):
        pass

    def plot_maze_weights(self):
        weights = np.zeros((self.m.rows, self.m.cols))
        data = self.transition_value
        for node, value in data.items():
            weights[node[0] - 1, node[1] - 1] = value

        plt.figure(figsize=(10, 8))

        if self.m.rows <= 30 and self.m.cols <= 30:
            for i in range(self.m.rows):
                for j in range(self.m.cols):
                    if self.__class__.__name__ == 'ValueIteration':
                        if self.m.rows >= 20 or self.m.cols >= 20:
                            plt.text(j, i, str(int(weights[i, j])), ha='center', va='center', color='black')
                        else:
                            plt.text(j, i, format(weights[i, j], '.2f'), ha='center', va='center', color='black')
                    else:
                        if self.m.rows >= 20 or self.m.cols >= 20:
                            plt.text(j, i, format(weights[i, j], '.1f'), ha='center', va='center', color='black')
                        else:
                            plt.text(j, i, format(weights[i, j], '.4f'), ha='center', va='center', color='black')

        plt.imshow(weights, cmap='cool', interpolation='nearest', origin='upper')
        plt.colorbar(label='Weight')
        plt.xticks(np.arange(self.m.cols), np.arange(1, self.m.cols + 1))
        plt.yticks(np.arange(self.m.rows), np.arange(1, self.m.rows + 1))
        plt.title(f'Maze Weights Visualization for {self.__class__.__name__} Algorithm in {self.m.rows}x{self.m.cols} Maze')
        plt.xlabel('Column')
        plt.ylabel('Row')
        plt.savefig(f'images/{self.__class__.__name__}-{self.m.rows}-{self.m.cols}.eps', format='eps')
        plt.show()


class ValueIteration(MarkovDecisionProcess):

    def __init__(self, m=None, goal=None):
        super().__init__(m, goal)
        self.transition_value = {node: 10 if node == self.goal else 0 for node in self.m.grid}
        self.transition_reward = {node: 100 if node == self.goal else 0 for node in self.m.grid}

    def execute_iteration(self):
        start = time.time()
        self.start_memory_tracing()
        values_converged_flag = False

        while not values_converged_flag:
            values_converged_flag = True

            for current_node in self.m.grid:
                temp_transition_value = []

                for direction in ['N', 'S', 'W', 'E']:
                    if self.m.maze_map[current_node][direction] == 1:
                        try:
                            if direction == 'N':
                                next_node = (current_node[0] - 1, current_node[1])
                            elif direction == 'S':
                                next_node = (current_node[0] + 1, current_node[1])
                            elif direction == 'W':
                                next_node = (current_node[0], current_node[1] - 1)
                            elif direction == 'E':
                                next_node = (current_node[0], current_node[1] + 1)
                        except IndexError:
                            next_node = None
                        if next_node is not None:
                            next_transition_value = self.transition_probability[direction] * (
                                    self.transition_reward[current_node] +
                                    self.discount_factor * self.transition_value[next_node])
                            temp_transition_value.append(next_transition_value)
                            self.transition_dictionary[current_node][direction] = next_transition_value
                best_transition_value = max(temp_transition_value)

                if abs(best_transition_value - self.transition_value[current_node]) > self.convergence_threshold:
                    values_converged_flag = False
                    self.transition_value[current_node] = best_transition_value

        end = time.time()
        self.stop_memory_tracing()
        self.cost_time = end - start

    def get_direction_for_current_node(self, current_node):
        return get_direction(self.transition_dictionary[current_node])


class PolicyIteration(MarkovDecisionProcess):

    def __init__(self, m=None, goal=None):
        super().__init__(m, goal)
        self.transition_value = {node: 1 if node == self.goal else 0 for node in self.m.grid}
        self.transition_reward = {node: 1 if node == self.goal else 0 for node in self.m.grid}
        self.policy = {node: None if node == self.goal else 'N' for node in self.m.grid}

    def calculate_transition_value(self, current_node):
        _temp_transition_value_ = {'N': 0, 'S': 0, 'W': 0, 'E': 0}
        temp_node_transition_value = {current_node: _temp_transition_value_}
        for direction in ['N', 'S', 'W', 'E']:
            if self.m.maze_map[current_node][direction] == 1:
                next_node = None
                try:
                    if direction == 'N':
                        next_node = (current_node[0] - 1, current_node[1])
                    elif direction == 'S':
                        next_node = (current_node[0] + 1, current_node[1])
                    elif direction == 'W':
                        next_node = (current_node[0], current_node[1] - 1)
                    elif direction == 'E':
                        next_node = (current_node[0], current_node[1] + 1)
                except IndexError:
                    next_node = None
                if next_node is not None:
                    next_transition_value = self.transition_value[next_node]
                else:
                    next_transition_value = 0

                temp_node_transition_value[current_node][direction] = self.transition_probability[direction] * (
                        self.transition_reward[current_node] +
                        self.discount_factor * next_transition_value)
        return temp_node_transition_value

    def execute_iteration(self):
        start = time.time()
        self.start_memory_tracing()
        value_converged_flag = False
        policy_converged_flag = False

        while not policy_converged_flag:
            policy_converged_flag = True
            value_converged_flag = False
            while not value_converged_flag:
                value_converged_flag = True
                for current_node in self.m.grid:
                    if current_node == self.goal:
                        continue
                    current_policy = self.policy[current_node]
                    if self.m.maze_map[current_node][current_policy] == 1:
                        next_node = None
                        try:
                            if current_policy == 'N':
                                next_node = (current_node[0] - 1, current_node[1])
                            elif current_policy == 'S':
                                next_node = (current_node[0] + 1, current_node[1])
                            elif current_policy == 'W':
                                next_node = (current_node[0], current_node[1] - 1)
                            elif current_policy == 'E':
                                next_node = (current_node[0], current_node[1] + 1)
                        except IndexError:
                            next_node = None
                        if next_node is not None:
                            next_transition_value = self.transition_value[next_node]
                        else:
                            next_transition_value = 0

                        self.transition_dictionary[current_node][current_policy] = (
                                self.transition_probability[current_policy] * (
                                self.transition_reward[current_node] +
                                next_transition_value * self.discount_factor))

                        if abs(self.transition_value[current_node] - (self.transition_probability[current_policy] * (
                                self.transition_reward[
                                    current_node] + next_transition_value * self.discount_factor))) > self.convergence_threshold:
                            self.transition_value[current_node] = self.transition_probability[current_policy] * (
                                    self.transition_reward[
                                        current_node] + next_transition_value * self.discount_factor)
                            value_converged_flag = False
            for current_node in self.m.grid:
                if current_node == self.goal:
                    continue
                current_node_transition_value = self.calculate_transition_value(current_node)
                current_node_transition_policy = get_direction(current_node_transition_value[current_node])
                if current_node_transition_policy != self.policy[current_node]:
                    self.policy[current_node] = current_node_transition_policy
                    policy_converged_flag = False

        self.cost_time = time.time() - start
        self.stop_memory_tracing()

    def get_direction_for_current_node(self, current_node):
        return self.policy[current_node]
