import customtkinter as ctk
from environment import Environment
import mapAlgorithms

COLOR_BACKGROUND = "#2B2B2B"
COLOR_GRID_BACKGROUND = "#3C3F41"
COLOR_EMPTY = "#D3D3D3"
COLOR_WALL = "#555555"
COLOR_START = "#FF0B0B"
COLOR_GOAL = "#76FF03"
COLOR_FRONTIER = "#ADD8E6"
COLOR_EXPLORED = "#FFFF99"
COLOR_PATH = "#1E90FF"
COLOR_BUTTON = "#4A4A4A"
COLOR_BUTTON_HOVER = "#5A5A5A"
COLOR_TEXT = "#FFFFFF"

class GridFrame(ctk.CTkFrame):
    """
    Manages the visual representation of the grid.
    """
    def __init__(self, master, environment: Environment, frame_size=600, **kwargs):
        super().__init__(master, **kwargs)
        self.environment = environment
        self.framesize = frame_size 
        self.cells = {} 
        self.cell_size = 0 
        self.grid_canvas_frame = None 

        self.configure(fg_color=COLOR_GRID_BACKGROUND)
        self._calculate_cell_dimens()
        self._draw_initial_grid()

    def _calculate_cell_dimens(self):
        num_rows = self.environment.rows    
        num_cols = self.environment.cols

        if num_cols == 0 or num_rows == 0:
            self.cell_size = 0
            self.actual_width = 0
            self.actual_height = 0
            self.padding_x = self.framesize / 2
            self.padding_y = self.framesize / 2
            return
        
        size_by_row = self.framesize / num_rows
        size_by_col = self.framesize / num_cols
        self.cell_size = min(size_by_col, size_by_row) * 0.95 

        self.actual_width = self.cell_size * num_cols
        self.actual_height = self.cell_size * num_rows

        self.padding_x = (self.framesize - self.actual_width) / 2
        self.padding_y = (self.framesize - self.actual_height) / 2
        
    def _draw_initial_grid(self):
        if self.grid_canvas_frame:
            self.grid_canvas_frame.destroy()

        self.grid_canvas_frame = ctk.CTkFrame(self, fg_color=COLOR_GRID_BACKGROUND, 
                                              width=self.actual_width, height=self.actual_height)
        self.grid_canvas_frame.place(x=self.padding_x, y=self.padding_y, anchor='nw')

        self.cells = {} 
        if self.cell_size == 0: return

        for r_idx in range(self.environment.rows):
            for c_idx in range(self.environment.cols):
                state = (c_idx, r_idx) 

                color = COLOR_EMPTY 
                if self.environment.is_wall(state):
                    color = COLOR_WALL
                elif self.environment.is_goal(state):
                    color = COLOR_GOAL
                elif state == self.environment.start:
                    color = COLOR_START
                
                cell_frame = ctk.CTkFrame(self.grid_canvas_frame, width=self.cell_size, height=self.cell_size,
                                          fg_color=color, border_width=1, 
                                          border_color=COLOR_BACKGROUND, 
                                          corner_radius=0) 
                cell_frame.place(x=c_idx * self.cell_size, y=r_idx * self.cell_size)
                self.cells[state] = cell_frame

    def reset_viz(self):
        self._draw_initial_grid() 
        self.update_idletasks()

    def update_cell_color(self, state: tuple, color: str, delay: int = 0):
        if state in self.cells:
            # Final path can overwrite anything except walls
            if color == COLOR_PATH: 
                 if not self.environment.is_wall(state):
                    self.cells[state].configure(fg_color=color)
            # Exploration or Frontier colors should not color start or goal
            elif color == COLOR_EXPLORED or color == COLOR_FRONTIER:
                if state != self.environment.start and not self.environment.is_goal(state):
                    if not self.environment.is_wall(state): 
                        self.cells[state].configure(fg_color=color)
            # Fallback for other colors (e.g. resetting to COLOR_EMPTY if needed, though reset_viz handles full reset)
            else:
                 if state != self.environment.start and not self.environment.is_goal(state):
                    if not self.environment.is_wall(state):
                        self.cells[state].configure(fg_color=color)
            
            if delay > 0:
                self.master.update_idletasks() 
                self.master.after(delay) 
                
    def animate_path(self, path_states: list, color: str, delay: int = 100):
        for state in path_states:
            if state in self.cells and not self.environment.is_wall(state):
                 self.cells[state].configure(fg_color=color)
                 if delay > 0:
                    self.master.update_idletasks()
                    self.master.after(delay)


class MapApp(ctk.CTk):
    def __init__(self, environment: Environment, **kwargs):
        super().__init__(**kwargs)
        self.environment = environment
        self.is_searching = False 

        self.title("Assignment 1 - Robot Navigation Visualization")
        self.geometry("900x700") 
        self.configure(fg_color=COLOR_BACKGROUND)
        ctk.set_appearance_mode("Dark")

        main_frame = ctk.CTkFrame(self, fg_color=COLOR_BACKGROUND)
        main_frame.pack(pady=20, padx=20, fill="both", expand=True)

        self.grid_display_frame_size = 600 
        self.grid_frame = GridFrame(main_frame, self.environment, 
                                    frame_size=self.grid_display_frame_size,
                                    width=self.grid_display_frame_size + 2*20, 
                                    height=self.grid_display_frame_size + 2*20)
        self.grid_frame.pack(side="left", padx=(0, 20), pady=0, fill="both", expand=True)

        controls_frame = ctk.CTkFrame(main_frame, width=250, fg_color=COLOR_BACKGROUND)
        controls_frame.pack(side="right", fill="y", padx=(10,0), pady=0)
        controls_frame.pack_propagate(False) 

        algo_label = ctk.CTkLabel(controls_frame, text="Search Algorithms", font=("Arial", 16, "bold"), text_color=COLOR_TEXT)
        algo_label.pack(pady=(10,10), padx=10, fill="x")

        self.algorithms = { 
            "DFS": {"func": mapAlgorithms.depth_first_search, "button": None},
            "BFS": {"func": mapAlgorithms.breadth_first_search, "button": None},
            "GBFS": {"func": mapAlgorithms.greedy_best_first_search, "button": None},
            "A*": {"func": mapAlgorithms.a_star_search, "button": None},
            "CUS1 (IDS)": {"func": mapAlgorithms.iterative_deepening_search, "button": None},
            "CUS2 (IDA*)": {"func": mapAlgorithms.ida_star_search, "button": None},
            "MAS (Multi-goal A*)": {"func": mapAlgorithms.multigoal_a_star, "button": None}
        }
        
        for algo_name, algo_data in self.algorithms.items():
            button = ctk.CTkButton(controls_frame, text=algo_name,
                                   command=lambda alg_func=algo_data["func"], alg_name=algo_name: self._run_search(alg_func, alg_name),
                                   fg_color=COLOR_BUTTON, hover_color=COLOR_BUTTON_HOVER, text_color=COLOR_TEXT,
                                   font=("Arial", 12))
            button.pack(pady=7, padx=10, fill="x")
            self.algorithms[algo_name]["button"] = button

        self.reset_button = ctk.CTkButton(controls_frame, text="Reset Grid",
                                     command=self._reset_search_and_viz,
                                     fg_color=COLOR_START, hover_color="#FF6060", text_color=COLOR_TEXT,
                                     font=("Arial", 12, "bold"))
        self.reset_button.pack(pady=20, padx=10, fill="x", side="bottom")

    def _toggle_buttons(self, enable: bool):
        for algo_data in self.algorithms.values():
            if algo_data["button"]:
                algo_data["button"].configure(state=ctk.NORMAL if enable else ctk.DISABLED)
        self.reset_button.configure(state=ctk.NORMAL if enable else ctk.DISABLED)

    def _viz_callback(self, state: tuple, status_key: str):
        color_map = {
            'frontier': COLOR_FRONTIER,
            'explored': COLOR_EXPLORED,
        }
        delay_map = {
            'frontier': 50,
            'explored': 50,
        }

        actual_color = color_map.get(status_key, COLOR_EMPTY) 
        actual_delay = delay_map.get(status_key, 0)
        
        self.grid_frame.update_cell_color(state, actual_color, delay=actual_delay)

    def _reset_search_and_viz(self):
        if self.is_searching:
            return
        self.grid_frame.reset_viz()

    def _run_search(self, algorithm_func, algorithm_name: str):
        if self.is_searching:
            return

        self.is_searching = True
        self._toggle_buttons(False) 
        
        print(f"Running {algorithm_name}...")
        self.grid_frame.reset_viz() 
        self.update_idletasks() 

        try:
            path_actions, nodes_count, goal_node_reached = algorithm_func(self.environment, viz=self._viz_callback)

            map_filename = "RobotNav-test.txt"  
            print(f"\n--- Result for {map_filename} using {algorithm_name} ---")
            if path_actions is not None:
                print(f"<Node {goal_node_reached}> {nodes_count}")
                if not path_actions and self.environment.is_goal(self.environment.start):
                    print("Already at goal")
                else:
                    print(f"{path_actions}") 
                    
                    path_states = [self.environment.start]
                    current_pos = self.environment.start
                    for action in path_actions:
                        if action == "UP": current_pos = (current_pos[0], current_pos[1] - 1)
                        elif action == "LEFT": current_pos = (current_pos[0] - 1, current_pos[1])
                        elif action == "DOWN": current_pos = (current_pos[0], current_pos[1] + 1)
                        elif action == "RIGHT": current_pos = (current_pos[0] + 1, current_pos[1])
                        path_states.append(current_pos)
                    
                    self.grid_frame.animate_path(path_states, COLOR_PATH, delay=100)
            else:
                print(f"No goal is reachable; {nodes_count}")
        except Exception as e:
            print(f"An error occurred during search execution for {algorithm_name}: {e}")
        finally:
            self.is_searching = False
            self._toggle_buttons(True) 


if __name__ == '__main__':
    try:
        env = Environment("RobotNav-test.txt") 
        app = MapApp(environment=env)
        app.mainloop()
    except FileNotFoundError:
        print("Error: Default map file 'RobotNav-test.txt' not found. "
              "Please ensure it's in the same directory as map.py or provide the correct path if running from elsewhere.")
    except Exception as e:
        print(f"An error occurred during GUI initialization: {e}")

