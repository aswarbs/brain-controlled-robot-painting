from Window import *

main_window = HostWindow()
main_window.change_frame("main menu")
main_window.master.mainloop()
"""
import csv
import matplotlib.pyplot as plt

data = []

with open('mapped_rotations.csv', newline='') as csvfile:
    csv_reader = csv.reader(csvfile)
    for row in csv_reader:
        data.append([float(val) for val in row])

# Transpose the data to separate y1, y2, and y3 values
data_transposed = list(zip(*data))

# Generate the x values based on the row index (starting from 0)
x = list(range(len(data_transposed[0])))

# Extract the three sets of y values
y1, y2, y3 = data_transposed

# Create a figure and axis
plt.figure()
# Plot the lines
plt.plot(x, y1, label='Alpha')
plt.plot(x, y2, label='Beta')
plt.plot(x, y3, label='Noise')

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Relative PSD')
plt.title('counting back PSD')

# Add a legend
plt.legend()

# Show the plot
plt.show()"""