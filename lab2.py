import heapq
import time
import psutil
import random

def heuristic(state):
    n = len(state)
    attacking_pairs = sum(state[i] == state[j] or abs(state[i] - state[j]) == abs(i - j) for i in range(n) for j in range(i + 1, n))
    return attacking_pairs

def generate_successors(state):
    successors = []

    for col, current_row in enumerate(state):
        for new_row in range(len(state)):
            if new_row != current_row:
                new_state = list(state)
                new_state[col] = new_row
                successors.append(new_state)

    return successors

def ids(initial_state, max_depth=10):
    depth = 0
    generated_states = 0
    closed_set = set()
    while depth <= max_depth:
        result, generated = dfs(initial_state, depth, closed_set)
        if result is not None:
            return result, generated, depth, initial_state, result
        generated_states += generated
        depth += 1
    return None, generated_states, max_depth, initial_state, None

def dfs(state, depth, closed_set):
    if depth == 0:
        if heuristic(state) == 0:
            return state, 1
        else:
            return None, 1

    generated_states = 0
    closed_set.add(tuple(state))  # Mark the current state as visited

    for successor in generate_successors(state):
        if tuple(successor) not in closed_set:
            result, generated = dfs(successor, depth - 1, closed_set)
            if result is not None:
                return result, generated + 1
            generated_states += generated

    return None, generated_states

def a_star(initial_state):
    open_list = [(heuristic(initial_state), 0, initial_state)]
    closed_set = set()
    generated_states = 0
    max_nodes_in_memory = 0  # Variable to store the maximum number of nodes in memory

    while open_list:
        f, g, state = heapq.heappop(open_list)
        if heuristic(state) == 0:
            return state, generated_states, max_nodes_in_memory, initial_state, state

        closed_set.add(tuple(state))  # Mark the current state as visited
        generated_states += 1

        if len(closed_set) > max_nodes_in_memory:
            max_nodes_in_memory = len(closed_set)

        for successor in generate_successors(state):
            if tuple(successor) not in closed_set:
                g_successor = g + 1
                f_successor = g_successor + heuristic(successor)
                heapq.heappush(open_list, (f_successor, g_successor, successor))

    return None, generated_states, max_nodes_in_memory, initial_state, None

def memory_usage():
    process = psutil.Process()
    return process.memory_info().rss

def generate_random_initial_state():
    return random.sample(range(8), 8)

def run_experiments(num_experiments=20, max_depth_ids=10):
    ids_times = []
    ids_generated_states = []
    ids_memory_usage = []

    a_star_times = []
    a_star_generated_states = []
    a_star_memory_usage = []
    a_star_max_nodes_in_memory = []

    for i in range(num_experiments):
        # IDS
        initial_state_ids = generate_random_initial_state()
        ids_start_time = time.time()
        result_ids, generated_ids, _, _, _ = ids(initial_state_ids, max_depth_ids)
        ids_end_time = time.time()
        ids_elapsed_time = ids_end_time - ids_start_time

        ids_times.append(ids_elapsed_time)
        ids_generated_states.append(generated_ids)
        ids_memory_usage.append(memory_usage())

        # A*
        initial_state_a_star = generate_random_initial_state()
        a_star_start_time = time.time()
        result_a_star, generated_a_star, max_nodes_a_star, _, _ = a_star(initial_state_a_star)
        a_star_end_time = time.time()
        a_star_elapsed_time = a_star_end_time - a_star_start_time

        a_star_times.append(a_star_elapsed_time)
        a_star_generated_states.append(generated_a_star)
        a_star_memory_usage.append(memory_usage())
        a_star_max_nodes_in_memory.append(max_nodes_a_star)

    # Print or return results after all experiments are completed
    for i in range(num_experiments):
        print("Experiment:", i + 1)
        print("Queens to Rearrange:", 1)
        print("Algorithm: IDS")
        print("Initial State:", initial_state_ids)
        print("Target State:", result_ids)
        print("Time:", ids_times[i], "seconds")
        print("Generated States:", ids_generated_states[i])
        print("Memory Usage:", ids_memory_usage[i] / 1024 / 1024, "MB")
        print()

        print("Experiment:", i + 1)
        print("Queens to Rearrange:", 1)
        print("Algorithm: A*")
        print("Initial State:", initial_state_a_star)
        print("Target State:", result_a_star)
        print("Time:", a_star_times[i], "seconds")
        print("Generated States:", a_star_generated_states[i])
        print("Memory Usage:", a_star_memory_usage[i] / 1024 / 1024, "MB")
        print("Max Nodes in Memory:", a_star_max_nodes_in_memory[i])
        print()

if __name__ == "__main__":
    run_experiments()
