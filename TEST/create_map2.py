width, height = 10, 10
max_value = 255
filename = 'map_050.pgm'

with open(filename, 'w') as f:
    f.write('P2\n')
    f.write(f'{width} {height}\n')
    f.write(f'{max_value}\n')
    for y in range(height):
        for x in range(width):
            if x == 9 and y == 0 :
                f.write('0 ')
            elif x == 0 and y == 0:
                f.write('0 ')
            else:
                f.write(f'{max_value} ')
        f.write('\n')
