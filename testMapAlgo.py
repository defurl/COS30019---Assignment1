import unittest
import os
from environment import Environment
import mapAlgorithms

TEST_DIR = "Test/"

class TestMapAlgorithms(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        """
        Set up the test environment by loading all test files.
        This method is called once for the entire test class.
        """
        self.small_map_start, self.small_map_goal, self.small_map_walls = Environment.parse_file("Test/small_map_functional.txt")
        self.medium_map_start, self.medium_map_goal, self.medium_map_walls = Environment.parse_file("Test/medium_map_obstacles.txt")
        self.large_map_start, self.large_map_goal, self.large_map_walls = Environment.parse_file("Test/large_map_functional.txt")
        self.find_all_goals_start, self.find_all_goals_goal, self.find_all_goals_walls = Environment.parse_file("Test/find_all_goals.txt")
        self.already_at_goal_start, self.already_at_goal_goal, self.already_at_goal_walls = Environment.parse_file("Test/already_at_goal.txt")
        self.no_goals_start, self.no_goals_goal, self.no_goals_walls = Environment.parse_file("Test/no_goals_reachable.txt")
        self.not_all_goals_start, self.not_all_goals_goal, self.not_all_goals_walls = Environment.parse_file("Test/not_all_goals_reachable.txt")

    def test_startPoint(self):
        """
        Test if the start point is correctly identified in the small map.
        """
        self.assertEqual(self.small_map_start, (0, 1), "Start point should be (0, 1)")

if __name__ == '__main__':
    if not os.path.exists(TEST_DIR):
        print(f"Warning: Test directory '{TEST_DIR}' not found.")
        print("Please create it and add your map files (e.g., small_map_functional.txt) there.")
    else:
        print(f"Running tests in directory: {TEST_DIR}")
    
    unittest.main(verbosity=2)