import os
import timeit
import matplotlib.pyplot as plt
import numpy as np
from environment import Environment
import mapAlgorithms

ALGORITHMS = {
    "DFS": mapAlgorithms.depth_first_search,
    "BFS": mapAlgorithms.breadth_first_search,
    "GBFS": mapAlgorithms.greedy_best_first_search,
    "A*": mapAlgorithms.a_star_search,
    # IDS currently runs very slow, consider taking out when testing
    "IDS (CUS1)": mapAlgorithms.iterative_deepening_search,
    "IDA* (CUS2)": mapAlgorithms.ida_star_search,
}

PERFORMANCE_FILE_PATH = "Test/Performance/"

def run_benchmarks(files_dir, files=None):
    """
    Runs benchmarks on all .txt files in the specified directory, or on specific files if provided.
    Args:
        files_dir (str): Path to the directory containing test map files.
        files (list, optional): A list of filenames to run. If None, runs all.
    Returns:
        tuple: (nodes, path, time, processed)
    """
    nodes = {name: [] for name in ALGORITHMS}
    path = {name: [] for name in ALGORITHMS}
    time = {name: [] for name in ALGORITHMS}
    processed = []

    if not os.path.exists(files_dir):
        print(f"Error: Performance test directory '{files_dir}' not found.")
        return nodes, path, time, processed

    if files is None:
        map_files = sorted([f for f in os.listdir(files_dir) if f.endswith('.txt')])
    else:
        map_files = files

    for map_file in map_files:
        file_path = os.path.join(files_dir, map_file)
        print(f"\n--- Benchmarking on: {map_file} ---")
        try:
            env = Environment(file_path)
            processed.append(map_file)

            for algo_name, algo_func in ALGORITHMS.items():
                start_time = timeit.default_timer()
                path_actions, nodes_count, goal_node = algo_func(env, viz=None) 
                end_time = timeit.default_timer()
                execution_time = end_time - start_time

                nodes[algo_name].append(nodes_count if nodes_count is not None else -1)
                path[algo_name].append(len(path_actions) if path_actions is not None else -1)
                time[algo_name].append(execution_time)

                print(f"    {algo_name}: Path Len={len(path_actions) if path_actions else 'N/A'}, Nodes={nodes_count}, Time={execution_time:.4f}s")

        except FileNotFoundError:
            print(f"  Skipping {map_file}, not found.")
        except Exception as e:
            print(f"  Error processing {map_file} with some algorithm: {e}")
            for algo_name_key in ALGORITHMS:
                if len(nodes[algo_name_key]) < len(processed):
                     nodes[algo_name_key].append(-1)
                     path[algo_name_key].append(-1)
                     time[algo_name_key].append(-1)

    return nodes, path, time, processed

def plot_results(results_dict, y_label, title_suffix, processed):
    """
    Generates and shows a bar plot for the given benchmark results.
    """
    if not processed:
        print(f"No files processed for plotting {title_suffix}.")
        return

    num_files = len(processed)
    algorithm_names = list(results_dict.keys())
    num_ALGORITHMS = len(algorithm_names)
    
    index = np.arange(num_files)
    bar_width = 0.8 / num_ALGORITHMS 

    fig, ax = plt.subplots(figsize=(max(12, num_files * 1.5), 7))

    for i, algo_name in enumerate(algorithm_names):
        values = results_dict.get(algo_name, [])
        plot_values = values[:num_files] if len(values) >= num_files else values + [-1] * (num_files - len(values))
        ax.bar(index + i * bar_width, plot_values, bar_width, label=algo_name)

    ax.set_xlabel('Map Files', fontweight='bold')
    ax.set_ylabel(y_label, fontweight='bold')
    ax.set_title(f'{y_label} by Algorithm ({title_suffix})', fontweight='bold')
    ax.set_xticks(index + bar_width * (num_ALGORITHMS -1) / 2)
    ax.set_xticklabels(processed, rotation=45, ha="right")
    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.6)
    fig.tight_layout()
    plt.show()

def main_benchmark():
    """
    Main function to run all benchmarks and generate plots.
    """
    print("Starting algorithm benchmarking...")
    
    general_files = [
        "alt_branching_maze.txt",
        "symmetry_map.txt",
        "tall_map.txt",
        "wide_map.txt",
        "tight_space_map.txt",
        "large_map_bottleneck.txt",
        "large_maze_like.txt",
    ]
    
    # Files to run individually due to large scale
    large_files = [
        "large_open_map.txt",
        "extra_large_open_map.txt",
        "extra_large_scattered.txt",
    ]

    # Benchmark general performance files
    nodes, paths, times, files = run_benchmarks(PERFORMANCE_FILE_PATH, files=general_files)
    
    if files: 
        plot_results(nodes, 'Nodes Created/Explored', 'General Maps', files)
        plot_results(paths, 'Path Length Found', 'General Maps', files)
        plot_results(times, 'Execution Time (seconds)', 'General Maps', files)
    else:
        print("No general map files were processed for benchmarking.")

    # Benchmark and plot large files individually
    for large_file in large_files:
        nodes_large, paths_large, times_large, files_large = run_benchmarks(PERFORMANCE_FILE_PATH, files=[large_file])
        if files_large:
            plot_results(nodes_large, 'Nodes Created/Explored', large_file, files_large)
            plot_results(paths_large, 'Path Length Found', large_file, files_large)
            plot_results(times_large, 'Execution Time (seconds)', large_file, files_large)
            
    print("\nBenchmarking complete.")

if __name__ == '__main__':
    if not os.path.exists(PERFORMANCE_FILE_PATH):
        os.makedirs(PERFORMANCE_FILE_PATH)
        print(f"Created directory: {PERFORMANCE_FILE_PATH}.")
        print(f"Please place your performance test map files there.")

    main_benchmark()
