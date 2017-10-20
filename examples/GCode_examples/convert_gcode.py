# shift midpoint and scale up or down a certain GCode file

file = open('Star2.nc', 'r')
lines = file.read().splitlines()
file.close()

offset = [49.749, 49.749]  # to place piece in the center
scaling = 0.5  # to scale piece up or down
offset_lines = []

for line in lines:
    split_line = line.split()
    for idx, s in enumerate(split_line):
        if 'X' in s:
            x_val = float(s[1:])
            x_val_n = x_val-offset[0]
            split_line[idx] = 'X' + str(x_val_n)
        if 'Y' in s:
            y_val = float(s[1:])
            y_val_n = y_val-offset[1]
            split_line[idx] = 'Y' + str(y_val_n)
    offset_lines.append(' '.join(split_line))

print offset_lines

new_lines = []

for line in offset_lines:
    split_line = line.split()
    for idx, s in enumerate(split_line):
        if 'X' in s:
            x_val = float(s[1:])
            x_val_n = x_val*scaling
            split_line[idx] = 'X' + str(x_val_n)
        if 'Y' in s:
            y_val = float(s[1:])
            y_val_n = y_val*scaling
            split_line[idx] = 'Y' + str(y_val_n)
        if 'I' in s:
            i_val = float(s[1:])
            i_val_n = i_val*scaling
            split_line[idx] = 'I' + str(i_val_n)
        if 'J' in s:
            j_val = float(s[1:])
            j_val_n = j_val*scaling
            split_line[idx] = 'J' + str(j_val_n)
    new_lines.append(' '.join(split_line))

file = open('Star2_shift_scale.nc', 'w')

for line in new_lines:
    file.write(line+'\n')

file.close()
