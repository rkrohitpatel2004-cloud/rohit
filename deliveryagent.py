import time
import heapq
import argparse

# Helper class for the Priority Queue used in UCS and A*
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return not self.elements

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class GridCity:
    """
    Represents the 2D grid environment the agent navigates.
    """
    def __init__(self, filename):
        self.static_obstacles = set()
        self.terrain_costs = {}
        self.start = None
        self.goal = None
        self._load_map(filename)

    def _load_map(self, filename):
        """Loads a map from a text file."""
        with open(filename, 'r') as f:
            for r, line in enumerate(f):
                for c, char in enumerate(line.strip()):
                    pos = (c, r)
                    if char == 'S':
                        self.start = pos
                        self.terrain_costs[pos] = 1
                    elif char == 'G':
                        self.goal = pos
                        self.terrain_costs[pos] = 1
                    elif char == 'X':
                        self.static_obstacles.add(pos)
                    elif char.isdigit():
                        self.terrain_costs[pos] = int(char)
                    elif char == '.':
                        self.terrain_costs[pos] = 1
        
        if not self.start or not self.goal:
            raise ValueError("Map must contain a 'S'tart and a 'G'oal location.")

    def is_obstacle(self, pos, dynamic_obstacles=None):
        """Checks if a given position is an obstacle."""
        if dynamic_obstacles is None:
            dynamic_obstacles = set()
        return pos in self.static_obstacles or pos in dynamic_obstacles

    def get_cost(self, pos):
        """Gets the movement cost for a given position."""
        return self.terrain_costs.get(pos, 1)

    def get_neighbors(self, pos, dynamic_obstacles=None):
        """
        Gets the valid neighbors for a position (4-connected).
        A neighbor is valid if it's not an obstacle.
        """
        x, y = pos
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_pos = (x + dx, y + dy)
            if new_pos in self.terrain_costs and not self.is_obstacle(new_pos, dynamic_obstacles):
                neighbors.append(new_pos)
        return neighbors

class DeliveryAgent:
    """
    The autonomous agent that finds and follows paths.
    """
    def __init__(self, environment):
        self.env = environment

    def ucs(self, start, goal, dynamic_obstacles=None):
        """Uniform-Cost Search algorithm."""
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {start: None}
        cost_so_far = {start: 0}
        nodes_expanded = 0

        while not frontier.empty():
            current = frontier.get()
            nodes_expanded += 1

            if current == goal:
                break

            for neighbor in self.env.get_neighbors(current, dynamic_obstacles):
                new_cost = cost_so_far[current] + self.env.get_cost(neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost
                    frontier.put(neighbor, priority)
                    came_from[neighbor] = current
        
        return self._reconstruct_path(came_from, start, goal), cost_so_far.get(goal), nodes_expanded

    def a_star(self, start, goal, dynamic_obstacles=None):
        """A* Search algorithm."""
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {start: None}
        cost_so_far = {start: 0}
        nodes_expanded = 0

        while not frontier.empty():
            current = frontier.get()
            nodes_expanded += 1

            if current == goal:
                break

            for neighbor in self.env.get_neighbors(current, dynamic_obstacles):
                new_cost = cost_so_far[current] + self.env.get_cost(neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._heuristic(neighbor, goal)
                    frontier.put(neighbor, priority)
                    came_from[neighbor] = current
        
        return self._reconstruct_path(came_from, start, goal), cost_so_far.get(goal), nodes_expanded

    def _heuristic(self, a, b):
        """Manhattan distance heuristic for A*."""
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def _reconstruct_path(self, came_from, start, goal):
        """Reconstructs the path from the came_from dictionary."""
        current = goal
        path = []
        if goal not in came_from: # No path found
            return []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

def print_results(path, cost, nodes, duration, algorithm_name):
    """Prints the formatted results of a search."""
    print(f"\n--- {algorithm_name} Results ---")
    if not path:
        print("No path found!")
    else:
        print(f"Path: {path}")
        print(f"Total Cost: {cost}")
    print(f"Nodes Expanded: {nodes}")
    print(f"Time Taken: {duration:.6f} seconds")
    print("--------------------------")

def simulate_dynamic_run(agent, environment):
    """
    Simulates a run where a dynamic obstacle appears, forcing the agent to replan.
    This fulfills the "proof-of-concept" requirement.
    """
    print("\n--- Running Dynamic Replanning Simulation ---")
    
    # 1. Agent creates an initial plan with A*
    print("[PLANNING] Agent is calculating the initial optimal path...")
    initial_path, initial_cost, _ = agent.a_star(environment.start, environment.goal)
    
    if not initial_path:
        print("[RESULT] No initial path could be found. Simulation aborted.")
        return

    print(f"[PLANNING] Initial path found with cost {initial_cost}: {initial_path}")

    # 2. Simulate agent moving along the path.
    #    An obstacle will "appear" halfway through the path.
    obstacle_position = initial_path[len(initial_path) // 2]
    dynamic_obstacles = {obstacle_position}
    
    print(f"\n[EXECUTION] Agent starts moving. A dynamic obstacle will appear at {obstacle_position}.")

    current_position = environment.start
    path_index = 1
    while path_index < len(initial_path):
        next_step = initial_path[path_index]
        
        # 3. Check for dynamic obstacles before moving.
        if next_step in dynamic_obstacles:
            print(f"\n[OBSTACLE] Dynamic obstacle detected at {next_step}! Agent must replan.")
            
            # 4. Replan from the current position with the new obstacle knowledge.
            print(f"[REPLANNING] Agent is replanning from current position {current_position}...")
            start_time = time.time()
            new_path, new_cost, nodes_expanded = agent.a_star(current_position, environment.goal, dynamic_obstacles)
            end_time = time.time()
            
            if not new_path:
                print("[RESULT] Agent could not find a new path around the obstacle. Delivery failed.")
                return

            print(f"[REPLANNING] New path found with cost {new_cost}: {new_path}")
            print_results(new_path, new_cost, nodes_expanded, end_time - start_time, "A* Replanning")
            print("\n[RESULT] Simulation finished successfully with replanning.")
            return

        # If no obstacle, continue moving.
        current_position = next_step
        path_index += 1

    print("[RESULT] Agent reached the goal without encountering the dynamic obstacle (this shouldn't happen in this simulation).")


def main():
    """Main function to run the agent from the command line."""
    parser = argparse.ArgumentParser(description="Run an autonomous delivery agent in a grid city.")
    parser.add_argument("map_file", type=str, help="Path to the map file (e.g., small_map.txt)")
    parser.add_argument("algorithm", type=str, choices=['ucs', 'astar', 'dynamic'], help="Algorithm to use: ucs, astar, or dynamic for simulation")
    
    args = parser.parse_args()

    # Load environment and create agent
    try:
        environment = GridCity(args.map_file)
        agent = DeliveryAgent(environment)
    except FileNotFoundError:
        print(f"Error: Map file not found at '{args.map_file}'")
        return
    except ValueError as e:
        print(f"Error loading map: {e}")
        return

    start_node = environment.start
    goal_node = environment.goal

    if args.algorithm == 'dynamic':
        simulate_dynamic_run(agent, environment)
    else:
        # Run the selected planning algorithm
        if args.algorithm == 'ucs':
            start_time = time.time()
            path, cost, nodes_expanded = agent.ucs(start_node, goal_node)
            end_time = time.time()
            algo_name = "Uniform-Cost Search"
        elif args.algorithm == 'astar':
            start_time = time.time()
            path, cost, nodes_expanded = agent.a_star(start_node, goal_node)
            end_time = time.time()
            algo_name = "A* Search"
        
        duration = end_time - start_time
        print_results(path, cost, nodes_expanded, duration, algo_name)

if __name__ == "__main__":
    main()
