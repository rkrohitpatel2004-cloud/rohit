import random
import argparse

def generate_map(width, height, num_obstacles):
    """Generates a random map grid."""
    # Start with a grid of standard terrain
    grid = [['.' for _ in range(width)] for _ in range(height)]

    # Place random obstacles
    for _ in range(num_obstacles):
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        grid[y][x] = 'X'
    
    # Place random terrain costs
    num_terrain = (width * height) // 5 # Cover 20% with varied terrain
    for _ in range(num_terrain):
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        if grid[y][x] == '.':
             grid[y][x] = str(random.randint(2, 5))

    # Place Start and Goal at random, ensuring they are not on an obstacle
    while True:
        start_x, start_y = random.randint(0, width - 1), random.randint(0, height - 1)
        if grid[start_y][start_x] != 'X':
            grid[start_y][start_x] = 'S'
            break
            
    while True:
        goal_x, goal_y = random.randint(0, width - 1), random.randint(0, height - 1)
        if grid[goal_y][goal_x] != 'X' and grid[goal_y][goal_x] != 'S':
            grid[goal_y][goal_x] = 'G'
            break

    return ["".join(row) for row in grid]

def save_map_to_file(grid_lines, filename):
    """Saves the generated grid to a file."""
    with open(filename, 'w') as f:
        for line in grid_lines:
            f.write(line + '\n')
    print(f"Map successfully saved to {filename}")

def main():
    parser = argparse.ArgumentParser(description="Generate a random map for the delivery agent.")
    parser.add_argument("width", type=int, help="Width of the map.")
    parser.add_argument("height", type=int, help="Height of the map.")
    parser.add_argument("output_filename", type=str, help="Name of the file to save the map to.")
    parser.add_gument("--obstacles", type=int, default=10, help="Number of obstacles to place.")
    
    args = parser.parse_args()
    
    if args.obstacles > args.width * args.height - 2:
        print("Error: Number of obstacles is too high for the given map dimensions.")
        return

    generated_grid = generate_map(args.width, args.height, args.obstacles)
    save_map_to_file(generated_grid, args.output_filename)

if __name__ == "__main__":
    main()
