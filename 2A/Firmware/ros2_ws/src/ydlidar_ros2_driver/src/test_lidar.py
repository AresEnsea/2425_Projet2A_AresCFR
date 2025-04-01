import numpy as np

# Generate x values from 0 to 2*pi
x = np.linspace(0, 2 * np.pi, 100)
# Compute the corresponding y values
y = np.sin(x)

# Save the data to a file
with open('data.dat', 'w') as f:
    for xi, yi in zip(x, y):
        f.write(f"{xi}\t{yi}\n")
