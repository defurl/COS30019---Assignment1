import heapq
from environment import Environment
from collections import deque
import copy 

def depth_first_search(environment: Environment, viz=None):
    """
    Performs Depth-First Search (DFS) on the given environment.

    Args:
        environment (Environment): The environment to search in.
        viz (function): Visualization function to call with state and status.
    Returns:
        tuple: A tuple containing the path actions, number of nodes created, and the goal state if found.
               If no goal is reachable, returns (None, number of nodes created, None).
    """
    start = environment.start
    parent = {start: (None, None)} 
    frontier = [start]    
    explored = set()     

    if environment.is_goal(start):
        if viz:
            viz(start, 'explored')
        return [], len(parent), start 
    
    while frontier:
        current = frontier.pop()  

        if current in explored:
            if viz:
                viz(current, 'explored') 
            continue
        explored.add(current)

        if environment.is_goal(current):
            if viz:
                viz(current, 'explored')
            path_actions = _reconstruct_path(parent, current, start)
            return path_actions, len(parent), current
        
        successors = environment.get_valid_successors(current) 
        for next, action in reversed(successors):
            if next not in explored: 
                frontier.append(next)
                parent[next] = (current, action)
                if viz:
                    viz(next, 'frontier')
        
        if viz:
            viz(current, 'explored')
                
    return None, len(parent), None 

def breadth_first_search(environment: Environment, viz=None):
    """
    Performs Breadth-First Search (BFS) on the given environment.

    Args:
        environment (Environment): The environment to search in.
        viz (function): Visualization function to call with state and status.
    Returns:
        tuple: A tuple containing the path actions, number of nodes created, and the goal state if found.
               If no goal is reachable, returns (None, number of nodes created, None).
    """
    start = environment.start
    parent = {start: (None, None)} 
    frontier = deque([start])  
    explored = set()

    if environment.is_goal(start):
        if viz:
            viz(start, 'explored')
        return [], len(parent), start 

    while frontier:
        current = frontier.popleft()  

        if environment.is_goal(current):
            if viz:
                viz(current, 'explored')
            path_actions = _reconstruct_path(parent, current, start)
            return path_actions, len(parent), current
        
        successors = environment.get_valid_successors(current) 
        for next, action in successors: 
            if next not in explored: 
                explored.add(next) 
                frontier.append(next)
                parent[next] = (current, action)
                if viz:
                    viz(next, 'frontier')
                
        if viz:
            viz(current, 'explored')
                
    return None, len(parent), None

def greedy_best_first_search(environment: Environment, viz=None):
    """
    Performs Greedy Best-First Search (GBFS) on the given environment.
    Args:
        environment (Environment): The environment to search in.
        viz (function): Visualization function to call with state and status.
    Returns:
        tuple: A tuple containing the path actions, number of nodes created, and the goal state if found.
               If no goal is reachable, returns (None, number of nodes created, None).
    """
    start = environment.start
    parent = {start: (None, None)}
    insertions = 0
    hcost = environment.heuristic(start)
    frontier_pq = [(hcost, insertions ,start)]  
    frontier_set = {start}
    # Use a set to track nodes in the frontier for O(1) lookups
    # This avoids reprocessing nodes already in the frontier.
    # This is important for the priority queue to work correctly.
    explored = set()

    insertions += 1 

    if environment.is_goal(start):
        if viz:
            viz(start, 'explored')
        return [], len(parent), start

    while frontier_pq:
        _, _, current = heapq.heappop(frontier_pq)  
        
        if current not in frontier_set: # Already processed
            continue
        frontier_set.remove(current)  

        if current in explored:
            if viz:
                viz(current, 'explored')
            continue
        explored.add(current)

        if environment.is_goal(current):
            if viz:
                viz(current, 'explored')
            path_actions = _reconstruct_path(parent, current, start)
            return path_actions, len(parent), current
        
        successors = environment.get_valid_successors(current)
        for next, action in successors:
            if next not in explored and next not in frontier_set:
                insertions += 1
                next_hcost = environment.heuristic(next)
                heapq.heappush(frontier_pq, (next_hcost, insertions, next))
                parent[next] = (current, action)
                frontier_set.add(next)
                if viz:
                    viz(next, 'frontier')
        
        if viz:
            viz(current, 'explored')

    return None, len(parent), None

def a_star_search(environment: Environment, viz=None, override_start: tuple = None, override_goal: set = None):
    """
    Performs A* search on the given environment.
    Args:
        environment (Environment): The environment to search in.
        viz (function): Visualization function to call with state and status.
        override_start (tuple): If provided, overrides the environment's start state. Used for MAS.
        override_goal (set): If provided, overrides the environment's goal set. Used for MAS.
    Returns:
        tuple: A tuple containing the path actions, number of nodes created, and the goal state if found.
               If no goal is reachable, returns (None, number of nodes created, None).
    """

    # override_start and override_goal are for MAS, standard A* uses environment's
    actual_start = override_start if override_start else environment.start
    
    # For MAS, the environment's goal set might be temporarily changed.
    # The heuristic should be calculated against the *current* set of goals for this A* run.
    # The environment object passed to heuristic() will have the correct .goal for this segment.

    parent = {actual_start: (None, None)}
    insertions = 0
    gcosts = {actual_start: 0} 
    
    hcost_start = environment.heuristic(actual_start)
    fcost_start = gcosts[actual_start] + hcost_start
    frontier_pq = [(fcost_start, insertions, actual_start)]
    insertions += 1
    
    frontier_set = {actual_start} 
    nodes = 1 

    if environment.is_goal(actual_start):
        if viz:
            viz(actual_start, 'explored')
        return [], nodes, actual_start
    
    while frontier_pq:
        _, _, current = heapq.heappop(frontier_pq)

        if current not in frontier_set: 
            continue
        frontier_set.remove(current)

        if environment.is_goal(current): 
            if viz:
                viz(current, 'explored')
            path_actions = _reconstruct_path(parent, current, actual_start)
            return path_actions, nodes, current
        
        current_gcost = gcosts[current]

        for next, action in environment.get_valid_successors(current):
            temp_gcost = current_gcost + 1 

            if temp_gcost < gcosts.get(next, float('inf')):
                if next not in gcosts: 
                    nodes +=1
                
                gcosts[next] = temp_gcost
                parent[next] = (current, action)
                
                hcost_next = environment.heuristic(next)
                fcost_next = temp_gcost + hcost_next
                
                heapq.heappush(frontier_pq, (fcost_next, insertions, next))
                insertions += 1
                frontier_set.add(next) 
                if viz:
                    viz(next, 'frontier')
        
        if viz:
            viz(current, 'explored')

    return None, len(parent), None

def iterative_deepening_search(environment: Environment, viz=None):
    """
    Performs Iterative Deepening Search (IDS) on the given environment.
    
    Args:
        environment (Environment): The environment to search in.
        viz (function): Visualization function to call with state and status.
    
    Returns:
        tuple: A tuple containing the path actions, number of nodes created, and the goal state if found.
               If no goal is reachable, returns (None, number of nodes created, None).
    """
    def depth_limited_search(environment: Environment, start: tuple, depth: int, viz=None):
        """
        Performs a depth-limited search up to a specified depth.
        Args:
            environment (Environment): The environment to search in.
            start (tuple): The starting state for the search.
            depth (int): The maximum depth to search.
            viz (function): Visualization function to call with state and status.
        """
        parent = {start: (None, None)}
        frontier = [(start, 0)]
        explored = set()
        
        if environment.is_goal(start):
            if viz:
                viz(start, 'explored')
            return [], len(explored), start
        while frontier:
            current, current_depth = frontier.pop()

            if current in explored:
                if viz:
                    viz(current, 'explored')
                continue
            explored.add(current)

            if environment.is_goal(current):
                if viz:
                    viz(current, 'explored')
                path_actions = _reconstruct_path(parent, current, start)
                return path_actions, len(explored), current
            
            if current_depth < depth:
                # 1. Get the raw successors list from the environment
                successors = environment.get_valid_successors(current)

                # --- NEW DETAILED DEBUG BLOCK ---
                print(f"\n--- DEBUGGING SUCCESSORS FOR NODE {current} ---")
                # 2. Print the RAW list. We expect this to start with UP.
                print(f"[RAW]      Successors list: {[(s[1]) for s in successors]}")
                
                # 3. Create the reversed list to see what it looks like.
                reversed_successors = list(reversed(successors))
                print(f"[REVERSED] Reversed list is: {[(s[1]) for s in reversed_successors]}")
                # --- END DEBUG BLOCK ---

                # 4. Loop through the reversed list as before
                for next_node, action in reversed_successors:
                    if next_node not in explored:
                        frontier.append((next_node, current_depth + 1))
                        if next_node not in parent:
                            parent[next_node] = (current, action)
                        if viz:
                            viz(next_node, 'frontier')

            if viz:
                viz(current, 'explored')

        print(explored)
        return None, len(explored), None
    
    start = environment.start
    max_depth = 0

    while True:
        path_actions, nodes, goal_state = depth_limited_search(environment, start, max_depth, viz)
        
        if goal_state is not None:
            return path_actions, nodes, goal_state
        
        max_depth += 1

def ida_star_search(environment: Environment, viz=None):
    """
    Performs Iterative Deepening A* (IDA*) search on the given environment.
    
    Args:
        environment (Environment): The environment to search in.
        viz (function): Visualization function to call with state and status.
    
    Returns:
        tuple: A tuple containing the path actions, number of nodes created, and the goal state if found.
               If no goal is reachable, returns (None, number of nodes created, None).
    """
    start = environment.start
    threshold = environment.heuristic(start)
    total_nodes = 0

    while True:
        parent = {start: (None, None)}
        gcosts = {start: 0}
        frontier = [(start, 0)] 

        min_next_threshold = float('inf')
        nodes = 1

        if viz and hasattr(viz.__self__, 'grid_frame'):
            viz.__self__.grid_frame.reset_viz()

        while frontier:
            current, gcost = frontier.pop() 

            if viz:
                viz(current, 'explored')

            if environment.is_goal(current):
                if viz:
                    viz(current, 'explored')
                path_actions = _reconstruct_path(parent, current, start)
                return path_actions, nodes, current

            for next, action in reversed(environment.get_valid_successors(current)):
                temp_gcost = gcost + 1
                fcost = temp_gcost + environment.heuristic(next)

                if fcost > threshold:
                    min_next_threshold = min(min_next_threshold, fcost)
                    continue

                if temp_gcost < gcosts.get(next, float('inf')):
                    if next not in gcosts:
                        nodes += 1

                    gcosts[next] = temp_gcost
                    parent[next] = (current, action)
                    
                    frontier.append((next, temp_gcost))

                    if viz:
                        viz(next, 'frontier')

            if viz:
                viz(current, 'explored')

        total_nodes += nodes

        if min_next_threshold == float('inf'):
            return None, total_nodes, None
        
        threshold = min_next_threshold

def multigoal_a_star(environment: Environment, viz=None):
    # Save original start and goal states
    original_start = environment.start
    original_goals = environment.goal.copy()
    
    if not original_goals:
        if original_start:
            if viz:
                viz(original_start, 'explored')
            return [], 1, None 
        return [], 0, None
    
    current = original_start
    unvisited_goals = original_goals.copy()
    
    actions = []
    total_nodes = 0
    last_visited = None
    
    # Check if start is already a goal
    if current in unvisited_goals:
        last_visited = current
        unvisited_goals.remove(current)
        if viz:
            viz(current, 'explored')
        if not unvisited_goals:
            return [], 1, current
    
    while unvisited_goals:
        # Temporarily set the environment's start and goal
        environment.start = current
        environment.goal = unvisited_goals
        
        # Run A* search for next goal
        next_actions, next_nodes, next_goal = a_star_search(environment, viz)
        total_nodes += next_nodes
        
        if next_actions is None:
            # Restore original environment state before returning
            environment.start = original_start
            environment.goal = original_goals
            return None, total_nodes, None
        
        actions.extend(next_actions)
        
        if next_goal not in unvisited_goals:
            # Restore original environment state before returning
            environment.start = original_start
            environment.goal = original_goals
            return None, total_nodes, None
        
        last_visited = next_goal
        unvisited_goals.remove(next_goal)
        current = next_goal
    
    # Restore original environment state
    environment.start = original_start
    environment.goal = original_goals
    
    if not unvisited_goals:
        return actions, total_nodes, last_visited
    else:
        return None, total_nodes, None

# ==================================== HELPER FUNCTIONS ====================================== #

def _reconstruct_path(parent: dict, goal_state: tuple, start_state: tuple) -> list:
    if goal_state == start_state:
        return [] 
    if goal_state != start_state and goal_state not in parent:
        return None 
    
    path_actions = []
    current = goal_state
    while current != start_state:
        prev_state_info = parent.get(current)
        if not prev_state_info or prev_state_info[0] is None: 
            if current == start_state: break
            return None 
        parent_s, action = prev_state_info
        path_actions.append(action)
        current = parent_s
    path_actions.reverse() 
    return path_actions

def print_path(actions: list, start: tuple):
    if actions is None: return []
    if not actions and start is not None: return [start]
    if start is None: return []
    
    path_states = [start]
    current = start
    
    for action in actions:
        if action == "UP": current = (current[0], current[1] - 1)
        elif action == "DOWN": current = (current[0], current[1] + 1)
        elif action == "LEFT": current = (current[0] - 1, current[1])
        elif action == "RIGHT": current = (current[0] + 1, current[1])
        else: return path_states 
        path_states.append(current)
    return path_states
