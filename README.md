Autonomous Delivery Agent
Project Overview
This project, based on CSA2001-Fundamentals of AI and ML, implements an autonomous agent that navigates a 2D grid city to deliver packages. The agent uses various search algorithms to find the most efficient path while considering static obstacles, variable terrain costs, and dynamic obstacles that require replanning.

Features
Grid Environment: The agent operates in a 2D grid where each cell can be a path, a static obstacle, or have a specific terrain cost.

Pathfinding Algorithms:

Uniform-Cost Search (UCS): Finds the cheapest path based on terrain costs.

A* Search: An informed search algorithm that uses the Manhattan distance heuristic to find the cheapest path more efficiently.

Dynamic Replanning: The agent can detect unexpected obstacles on its planned path and recalculate a new route from its current position.

Command-Line Interface: A CLI to run simulations with different maps and algorithms.

File Structure
autonomous_agent.py: The main Python script containing all the code for the environment, agent, search algorithms, and simulation logic.

small_map.txt, medium_map.txt, large_map.txt, dynamic_map.txt: Pre-generated map files for testing.

map_generator.py: A Python script to generate custom random maps.

README.md: This file.

Map File Format
The grid maps are represented as simple text files:

S: Start position of the agent.

G: Goal (delivery destination).

X: A wall or static obstacle (impassable).

. or 1: Standard terrain with a movement cost of 1.

2-9: Difficult terrain with a corresponding movement cost.

D: A dynamic obstacle used for replanning simulations.

Prerequisites
Python 3.6 or higher.

No external libraries are required.

How to Run
Clone or download the repository.

Open your terminal or command prompt and navigate to the project directory.

Running a Specific Algorithm
You can run a specific algorithm on a chosen map using the following command:

python autonomous_agent.py <map_file> <algorithm>

Arguments:

<map_file>: The path to the map file (e.g., small_map.txt).

<algorithm>: The search algorithm to use (ucs or astar).

Example:

python autonomous_agent.py medium_map.txt astar

This will run the A* algorithm on the medium map and print the results, including the path taken, total cost, number of nodes expanded, and execution time.

Running the Dynamic Replanning Simulation
To see the agent's replanning capabilities in action, run the dynamic simulation. In this mode, the agent first plans a path. During execution, a 'dynamic' obstacle appears, forcing the agent to find a new path.

python autonomous_agent.py dynamic_map.txt dynamic

This will output logs showing the initial plan, the point where an obstacle is detected, and the new plan that is formulated.

Generating New Maps
You can create your own maps using the map_generator.py script.

python map_generator.py <width> <height> <output_filename> [--obstacles <num>]

Example:

python map_generator.py 30 20 my_map.txt --obstacles 50

This creates a 30x20 map named my_map.txt with 50 random obstacles. You can then edit the file to place the S (start) and G (goal) markers.