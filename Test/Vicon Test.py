import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages
import numpy as np

def extract_numbers_txt(line):
    result = []
    temp = ""
    for char in line:
        if char.isdigit() or char == '.' or char == '-':  # Check if the character is part of a number
            temp += char
        else:
            if temp:  # If a number was being built, add it to the result
                result.append(float(temp))  # Convert to float for plotting
                temp = ""
    if temp:  # Add the last number if it exists
        result.append(float(temp))  # Convert to float for plotting
    return result

def txt():
    columns = []
    with open(txt_path, 'r') as file:
        lines = file.readlines()[:-3]
        for line in lines:
            numbers = extract_numbers_txt(line)
            # Ensure the columns list is large enough to hold all numbers
            while len(columns) < len(numbers):
                columns.append([])
            # Append each number to its respective column
            for i, number in enumerate(numbers):
                columns[i].append(number)
    return columns[2:], columns[:2]

def mcap():
    columns = []
    for msg in read_ros2_messages(mcap_path):
        data = msg.ros_msg.data
        for i, value in enumerate(data):
            if len(columns) <= i:
                columns.append([])  # Add a new list for this index
            columns[i].append(value)
    return columns[2:]

def compare(txt_coloms, mcap_coloms):
    success = 0
    total = 0
    for i in range(len(mcap_coloms)):
        for j in range(len(mcap_coloms[i])):
            if txt_coloms[i][j] == mcap_coloms[i][j]:
                success += 1
            total += 1
    print(f"Success: {success} ; Total: {total} ; Success rate: {success / total * 100}%")

def plot_columns(columns):
    column_labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
    y_axis = ['Position[m]', 'Angle[rad]']
    for i, column in enumerate(columns):
        plt.figure()
        plt.plot(column, marker='o', linestyle='-', markersize=1, label=column_labels[i])
        plt.title(f'Plot of {column_labels[i]}')
        plt.xlabel('Time(s)')
        plt.ylabel(y_axis[i//3])
        plt.legend()
        plt.grid(True)
    plt.show()

def plot(columns):
    column_labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
    y_axis = ['Position[m]', 'Angle[rad]']
    
    # Create a 2x3 grid for all six plots
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()  # Flatten the 2D array of axes for easier indexing
    
    for i, column in enumerate(columns):
        axes[i].plot(column, marker='o', linestyle='-', markersize=1, label=column_labels[i])
        axes[i].set_title(f'Plot of {column_labels[i]}')
        axes[i].set_xlabel('Package number')
        axes[i].set_ylabel(y_axis[i // 3])  # Use 'Position[m]' for first 3, 'Angle[rad]' for last 3
        axes[i].legend()
        axes[i].grid(True)
    
    plt.tight_layout()  # Adjust layout to prevent overlap
    plt.show()

def get_data(within_index, outside_index, coloms):
    within_values = [[] for _ in coloms]
    outside_values = [[] for _ in coloms]
    within_std = []

    for i, colom in enumerate(coloms):
        within_values[i] = [colom[j] for j in within_index]
        outside_values[i] = [colom[k] for k in outside_index]
        within_std.append(np.std(within_values[i]))

    return within_values, outside_values, within_std

def get_time(time, outside_indices):
    m = 0
    s = 0
    ms_prev = -1
    start_ms = time[1][0]

    for i, ms in enumerate(time[1]):
        ms = ms - start_ms
        if ms < ms_prev:
            s += 1
            if s >= 60:
                s = 0
                m += 1
        ms_prev = ms
        if i == outside_indices[-1]:
            outside_end_s = s
            outside_end_ms = ms
        if i == outside_indices[0]:
            outside_start_s = s
            outside_start_ms = ms

    outside_time_start = outside_start_s + outside_start_ms / 1000
    outside_time_end = outside_end_s + outside_end_ms / 1000
    outside_time = outside_time_end - outside_time_start

    time_s = m*60 + s/100 + ms/100000
    
    return time_s, outside_time


txt_path = "RosBags\\ViconTest.txt"
mcap_path = "RosBags\\ViconTest0605\\ViconTest0605_0.mcap"


txt_coloms, time = txt()
mcap_coloms = mcap()

mean_txt = np.mean(txt_coloms[0])
threshold = 0.01
within_indices = [i for i, value in enumerate(txt_coloms[0]) if (mean_txt - threshold) <= value <= (mean_txt + threshold)]
outside_indices = [i for i in range(len(txt_coloms[0])) if i not in within_indices]

within_values, outside_values, within_std = get_data(within_indices, outside_indices, txt_coloms)

time_s, outside_time = get_time(time, outside_indices)

within_means = [np.mean(values) for values in within_values]
outside_means = [np.mean(values) for values in outside_values]


print("Comparing recorded Vicon data to recived data")
compare(txt_coloms, mcap_coloms)
print(f"Total time: {time_s} seconds")
print("")

print(f"Error interval: {outside_time} seconds")


print("Within interval:")
print(f"x standard deviation: {within_std[0]}")
print(f"y standard deviation: {within_std[1]}")
print(f"z standard deviation: {within_std[2]}")
print(f"roll standard deviation: {within_std[3]}")
print(f"pitch standard deviation: {within_std[4]}")
print(f"yaw standard deviation: {within_std[5]}")

plot(txt_coloms)