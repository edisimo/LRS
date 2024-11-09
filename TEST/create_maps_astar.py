# generate_maps.py

def create_map(filename, obstacles):
    width, height = 10, 10
    max_value = 255

    with open(filename, 'w') as f:
        f.write('P2\n')
        f.write(f'{width} {height}\n')
        f.write(f'{max_value}\n')
        for y in range(height):
            for x in range(width):
                if (x, y) in obstacles:
                    f.write('0 ')  # Obstacle
                else:
                    f.write(f'{max_value} ')  # Free space
            f.write('\n')

# Map at z = 0.0
obstacles_z0 = {(5, y) for y in range(10)}  # Vertical wall at x=5
create_map('map_000.pgm', obstacles_z0)

# Map at z = 0.5
obstacles_z05 = {(x, 5) for x in range(10)}  # Horizontal wall at y=5
create_map('map_050.pgm', obstacles_z05)

# Map at z = 1.0
obstacles_z1 = {(x, x) for x in range(10)}  # Diagonal wall from (0,0) to (9,9)
create_map('map_100.pgm', obstacles_z1)
