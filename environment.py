import ast

class Environment:
    """
    Represents the grid environment for the assignment
    Parses the file, stores map details, provides helper functions
    """
    def __init__(self, filename):
        """
        Initializes environment, loads and parses map file
        Args:
            filename (str): path to map file
        """
        self.rows = 0
        self.cols = 0
        self.start = None # (col, row) tuple
        self.goal = set()  # Set of (col, row) tuples
        self.walls = set()      # Set of (col, row) tuples for wall locations

        self.parse_file(filename)

    def parse_file(self, filename):
        """
        Parses the input file to extract environment attributes.
        Assumes the input file format is 100% correct.
        Args:
            filename (str): Path to the input file.
        Raises:
            FileNotFoundError: If the file does not exist.
            Exception: If parsing fails due to incorrect format not caught by ast.literal_eval.
        """
        try:
            with open(filename, 'r') as file_handle:
                # Line 1: Grid dimensions [N, M] -> N=rows, M=cols
                # Sample: [5,11]
                self.rows, self.cols = ast.literal_eval(file_handle.readline().strip())

                # Line 2: Start state (x, y) -> (col, row)
                # Sample: (0,1)
                self.start = ast.literal_eval(file_handle.readline().strip())

                # Line 3: Goal states (xG1,yG1)|(xG2,yG2)|...
                # Sample: (7,0) | (10,3)
                goals_line = file_handle.readline().strip()
                if goals_line:
                    goal_parts_str = goals_line.split('|')
                    for part_str in goal_parts_str:
                        cleaned_part_str = part_str.strip() # Handles spaces around '|'
                        self.goal.add(ast.literal_eval(cleaned_part_str))
                
                # Walls (x,y,w,h) -> (start_col, start_row, width, height)
                # Sample: (2,0,2,2)
                for line_content in file_handle:
                    line_content = line_content.strip()
                    if not line_content: continue # Skip empty lines if any

                    start_col, start_row, width, height = ast.literal_eval(line_content)
                    for r_offset in range(height):
                        for c_offset in range(width):
                            self.walls.add((start_col + c_offset, start_row + r_offset))
        
        except FileNotFoundError:
            print(f"Error: The map file '{filename}' was not found.")
            raise 
        except Exception as e: # Catch other potential parsing errors
            print(f"Error parsing map file '{filename}': {e}")
            raise

    def is_within_bounds(self, state):
        col, row = state
        return 0 <= col < self.cols and 0 <= row < self.rows

    def is_wall(self, state):
        return state in self.walls

    def get_valid_successors(self, current_state):
        """
        Returns a list of valid successor states and the actions to reach them,
        in the required order: UP, LEFT, DOWN, RIGHT.
        """
        successors = []
        col, row = current_state
        possible_moves = [
            (0, -1, "UP"), 
            (-1, 0, "LEFT"), 
            (0, 1, "DOWN"), 
            (1, 0, "RIGHT")
        ]

        for d_col, d_row, action in possible_moves:
            next_col, next_row = col + d_col, row + d_row
            next_state = (next_col, next_row)
            if self.is_within_bounds(next_state) and not self.is_wall(next_state):
                successors.append((next_state, action))
                
        return successors

    def manhattan_distance(self, state1, state2):
        return abs(state1[0] - state2[0]) + abs(state1[1] - state2[1])
    
    def heuristic(self, state):
        if not self.goal:
            return float('inf')
        min_dist = float('inf')
        for goal in self.goal:
            dist = self.manhattan_distance(state, goal)
            if dist < min_dist:
                min_dist = dist
        return min_dist
    
    def is_goal(self, state):
        return state in self.goal
