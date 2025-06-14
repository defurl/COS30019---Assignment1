import unittest
import os
from environment import Environment # Your Environment class
import mapAlgorithms # Your search algorithms

# Define a base path for test files. Assumes test files are in a 'Test' subdirectory.
BASE_TEST_FILE_PATH = "Test"

class TestMapAlgorithms(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """
        Set up method to load environments once for all tests.
        Creates the Test directory if it doesn't exist.
        """
        if not os.path.exists(BASE_TEST_FILE_PATH):
            os.makedirs(BASE_TEST_FILE_PATH)
            print(f"Created directory: {BASE_TEST_FILE_PATH}.")
            print(f"Please place your test map files (e.g., small_map_functional.txt) in the '{BASE_TEST_FILE_PATH}' directory.")
        
        def get_test_file(filename):
            path = os.path.join(BASE_TEST_FILE_PATH, filename)
            if not os.path.exists(path):
                print(f"Warning: Test file {path} not found. Some tests may fail or be skipped.")
            return path

        try:
            # Using your friend's map names for consistency in testing
            cls.small_env = Environment(get_test_file("small_map_functional.txt"))
        except FileNotFoundError:
            cls.small_env = None
            print("Warning: Could not load 'small_map.txt' for testing.")

        try:
            cls.no_goal_env = Environment(get_test_file("no_goals_reachable.txt"))
        except FileNotFoundError:
            cls.no_goal_env = None
            print("Warning: Could not load 'none_goal.txt' for testing.")

        try:
            cls.at_goal_env = Environment(get_test_file("already_at_goal.txt"))
        except FileNotFoundError:
            cls.at_goal_env = None
            print("Warning: Could not load 'already_at_goal.txt' for testing.")

        try:
            cls.multi_goal_success_env = Environment(get_test_file("find_all_goals.txt"))
        except FileNotFoundError:
            cls.multi_goal_success_env = None
            print("Warning: Could not load 'find_all_goal.txt' for MAS success testing.")
        
        try:
            cls.multi_goal_fail_env = Environment(get_test_file("not_all_goals.txt"))
        except FileNotFoundError:
            cls.multi_goal_fail_env = None
            print("Warning: Could not load 'not_all_goal.txt' for MAS failure testing.")

    def test_01_parsing(self):
        """Test parsing of a sample map file."""
        if not self.small_env:
            self.skipTest("small_map.txt not loaded.")
        
        # !! These asserts should match the content of YOUR "small_map.txt" !!
        # Using values from your friend's test file as a guide.
        self.assertEqual(self.small_env.start, (0,0), "Small map start position incorrect.")
        self.assertEqual(self.small_env.goal, {(5,1), (5,3)}, "Small map goal positions incorrect.")
        self.assertTrue((1,0) in self.small_env.walls, "Small map missing wall at (1,0).")
        self.assertFalse((0,1) in self.small_env.walls, "Small map incorrectly has wall at (0,1).")

    def _run_and_assert_algorithm(self, algorithm_func, env, expected_path, 
                                  expected_nodes, expected_goal, name):
        if not env:
            self.skipTest(f"Environment for {name} not loaded.")

        path_actions, nodes_count, goal_node = algorithm_func(env, gui_callback=None)
        
        self.assertEqual(path_actions, expected_path, f"{name}: Path mismatch")
        self.assertEqual(nodes_count, expected_nodes, f"{name}: Node count mismatch")
        self.assertIn(goal_node, env.goal, f"{name}: Reached goal {goal_node} is not a valid goal in {env.goal}")


    def test_02_small_map_pathfinding(self):
        """Test all algorithms on the small map."""
        if not self.small_env:
            self.skipTest("small_map.txt not loaded.")
        
        # !!! IMPORTANT: Replace these placeholder values with your own verified results !!!
        expected_results = {
            "DFS":  {'path': ['right', 'right', 'down', 'down', 'down', 'right', 'right', 'up', 'up', 'up', 'right', 'down'], 'nodes': 18, 'goal': (5,1)},
            "BFS":  {'path': ['right', 'right', 'right', 'right', 'down', 'right'], 'nodes': 17, 'goal': (5,1)},
            "GBFS": {'path': ['right', 'right', 'right', 'right', 'down', 'right'], 'nodes': 11, 'goal': (5,1)},
            "A*":   {'path': ['right', 'right', 'right', 'right', 'down', 'right'], 'nodes': 11, 'goal': (5,1)},
            "IDS":  {'path': ['right', 'right', 'right', 'right', 'down', 'right'], 'nodes': 13, 'goal': (5,1)},
            "IDA*": {'path': ['right', 'right', 'right', 'right', 'down', 'right'], 'nodes': 11, 'goal': (5,1)},
        }
        
        test_cases = {
            "DFS": mapAlgorithms.depth_first_search,
            "BFS": mapAlgorithms.breadth_first_search,
            "GBFS": mapAlgorithms.greedy_best_first_search,
            "A*": mapAlgorithms.a_star_search,
            "IDS": mapAlgorithms.iterative_deepening_search,
            "IDA*": mapAlgorithms.ida_star_search,
        }

        for name, func in test_cases.items():
            with self.subTest(algorithm=name):
                # We need to assertIn for goal, since multiple goals exist
                path_actions, nodes_count, goal_node = func(self.small_env, gui_callback=None)
                self.assertEqual(path_actions, expected_results[name]['path'])
                self.assertEqual(nodes_count, expected_results[name]['nodes'])
                self.assertEqual(goal_node, expected_results[name]['goal']) # Check for the specific goal reached

    def test_03_already_at_goal(self):
        if not self.at_goal_env:
            self.skipTest("already_at_goal.txt not loaded.")
        
        for name, algo_func in ALGORITHMS_TO_TEST.items():
            with self.subTest(algorithm=name):
                path, nodes, goal = algo_func(self.at_goal_env)
                self.assertEqual(path, [], f"{name}: Path should be empty.")
                self.assertEqual(nodes, 1, f"{name}: Node count should be 1.")
                self.assertIn(goal, self.at_goal_env.goal, f"{name}: Goal node is incorrect.")

    def test_04_unreachable_goal(self):
        if not self.no_goal_env:
            self.skipTest("none_goal.txt not loaded.")
        
        for name, algo_func in ALGORITHMS_TO_TEST.items():
            with self.subTest(algorithm=name):
                path, nodes, goal = algo_func(self.no_goal_env)
                self.assertIsNone(path, f"{name}: Path should be None.")
                self.assertIsNone(goal, f"{name}: Goal node should be None.")

    def test_05_mas_success(self):
        if not self.multi_goal_success_env:
            self.skipTest("find_all_goal.txt not loaded.")
        
        # !!! FILL WITH YOUR VERIFIED RESULTS FOR MAS !!!
        EXPECTED_PATH = ['down', 'down', 'down', 'right', 'right', 'right', 'up', 'up', 'up', 'left', 'right', 'right', 'right', 'up', 'right', 'right', 'down', 'down', 'down', 'down', 'down', 'left', 'left', 'up', 'up']
        EXPECTED_NODES = 100 # Placeholder
        
        path, nodes, goal = mapAlgorithms.multigoal_a_star(self.multi_goal_success_env)
        
        self.assertEqual(path, EXPECTED_PATH, "MAS Success: Path is incorrect.")
        self.assertEqual(nodes, EXPECTED_NODES, "MAS Success: Node count is incorrect.")
        self.assertIsNotNone(goal, "MAS Success: Final goal should not be None.")
        # Check if all goals were visited. The final path should end at one of them.
        self.assertIn(goal, self.multi_goal_success_env.goal, "MAS Success: Final node is not a valid goal.")

    def test_06_mas_failure(self):
        if not self.multi_goal_fail_env:
            self.skipTest("not_all_goal.txt not loaded.")
        
        path, nodes, goal = mapAlgorithms.multigoal_a_star(self.multi_goal_fail_env)
        
        self.assertIsNone(path, "MAS Failure: Path should be None.")
        self.assertIsNone(goal, "MAS Failure: Final goal should be None.")

# Helper dictionary for tests 3 and 4
ALGORITHMS_TO_TEST = {
    "DFS": mapAlgorithms.depth_first_search,
    "BFS": mapAlgorithms.breadth_first_search,
    "GBFS": mapAlgorithms.greedy_best_first_search,
    "A*": mapAlgorithms.a_star_search,
    "IDS": mapAlgorithms.iterative_deepening_search,
    "IDA*": mapAlgorithms.ida_star_search
}

if __name__ == '__main__':
    unittest.main(verbosity=2)
