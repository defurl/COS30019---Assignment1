import sys

from environment import Environment
from mapAlgorithms import *

def main():
    if len(sys.argv) < 3:
        print("Usage: python search.py <map_file> <method>")
        print("Available methods: DFS, BFS, GBFS, A*, CUS1, CUS2, MAS") 
        sys.exit(1)

    file_name = sys.argv[1]
    method_arg = sys.argv[2].upper() 

    try:
        env = Environment(file_name)

    except FileNotFoundError:
        print(f"Error: File '{file_name}' was not found.")
        sys.exit(1)
    except ValueError as e: 
        print(f"Error processing map file '{file_name}': {e}")
        sys.exit(1)
    except Exception as e: 
        print(f"An unexpected error occurred while setting up the environment for '{file_name}': {e}")
        sys.exit(1)
    
    path_actions_list = None 
    nodes_count = 0
    goal_node_state = None

    if method_arg == "DFS":
        path_actions_list, nodes_count, goal_node_state = depth_first_search(env)
    elif method_arg == "BFS":
        path_actions_list, nodes_count, goal_node_state = breadth_first_search(env)
    elif method_arg == "GBFS":
        path_actions_list, nodes_count, goal_node_state = greedy_best_first_search(env)
    elif method_arg == "A*":
        path_actions_list, nodes_count, goal_node_state = a_star_search(env)
    elif method_arg == "CUS1":
        path_actions_list, nodes_count, goal_node_state = iterative_deepening_search(env) 
    elif method_arg == "CUS2":
        path_actions_list, nodes_count, goal_node_state = ida_star_search(env)
    elif method_arg == "MAS":
        path_actions_list, nodes_count, goal_node_state = multigoal_a_star(env)
    else:
        print(f"Error: Method '{method_arg}' is not implemented or unknown.")
        print("Available methods: DFS, BFS, GBFS, A*, CUS1, CUS2, MAS") 
        sys.exit(1)
    
    print(f"\n--- Result for {file_name} using {method_arg} ---")
    if path_actions_list is not None:
        print(f"<Node {goal_node_state}> {nodes_count}")
        if not path_actions_list and env.is_goal(env.start): 
            print("Already at the goal") 
        else:
            print(path_actions_list) 
            print_path(path_actions_list, env.start)
    else:
        print(f"No goal is reachable; {nodes_count}")

if __name__ == "__main__":
    main()
