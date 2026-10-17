import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import math
# import random
import csv
# import pandas as pd
# import pywinctl as pwc
# import mss
# import time
# import pickle


def make_3D_plot(csv_files, n, title="Obstacle Avoidance", file_name="graph100.png"):
    """
    Plots the specified x, y, and z columns from one or more CSV files.

    csv_files (list of str): Paths to the CSV files
    x_column (str): Name of the column to use as the x-axis
    y_column (str): Name of the column to use as the y-axis
    labels (list of str, optional): Labels for each CSV file's plot
    title (str, optional): Title of the plot
    xlabel (str, optional): Label for the x-axis
    ylabel (str, optional): Label for the y-axis
    file_name (str, optional): If provided, saves the figure with this file name
    """
    # if y_labels is None:
    #     y_labels = y_columns


    # Create a figure and a 3D axes object
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x_vals = []
    y_vals = []
    z_vals = []

    ball_x = []
    ball_y = []

    t_vals = []
    x_vals = []
    y_vals = []

    april_tag_x = []
    april_tag_y = []


    # fig, ax1 = plt.subplots(figsize=(10, 6))

    for file in csv_files:
        with open(file) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            print(csv_reader)

            line = 0

            for row in csv_reader:
                # print(line)
                if line != 0:
                    # z_vals.append(float(row[0]))
                    # for i in range(n):
                    # while row is not None
                    t_vals.append(float(row[0]))
                    x_vals.append(float(row[1]))
                    y_vals.append(float(row[2]))

                    # y_vals.append(float(row[1]))
                    # x_vals.append(float(row[2]))
                    
                    ball_x.append(float(row[7]))
                    ball_y.append(float(row[8]))
                    april_tag_x.append(1)
                    april_tag_y.append(-0.5)
                    line = line + 1
                else:
                    line = line + 1
        # label = labels[i] if labels else f"File {i+1}"
        # ax.plot(x_vals, y_vals, label=label)

        # if len(y_columns) == 2:
        #     if i == 0:
        #         ax2 = ax1.twinx()
        #     ax2.plot(x_vals, y2_vals, linestyle='--')

    # Plot the 3D scatter plot
    # ax.scatter3D(x_vals, y_vals, z_vals, c='r', marker='o')
    ax.scatter3D(x_vals, y_vals, t_vals, c=t_vals, cmap='Reds', marker='o', label='KF')
    ax.scatter3D(ball_x, ball_y, t_vals, c=t_vals, cmap='Blues', marker='o', label='Ball')
    ax.scatter3D(april_tag_x, april_tag_y, t_vals, c=t_vals)

    x_vals = np.array(x_vals)
    y_vals = np.array(y_vals)

    ball_x = np.array(ball_x)
    ball_y = np.array(ball_y)

    error_x = ball_x - x_vals
    error_y = ball_y - y_vals


    # Set labels for the axes
    ax.set_xlabel('x-axis')
    ax.set_ylabel('y-axis')
    ax.set_zlabel('t-axis')

    # Set a title for the plot
    ax.set_title(title)

    # Display the plot
    plt.legend()
    plt.savefig("SUPER_NICE_CODE/3Dpath.png")
    plt.show()

    plt.plot(t_vals, error_x)
    plt.axhline(0.0762, color='red', linestyle='--')
    plt.axhline(-0.0762, color='red', linestyle='--')
    plt.title("X Position Error [m]")
    plt.savefig("SUPER_NICE_CODE/x_positional_error.png")
    plt.show()

    plt.plot(t_vals, error_y)
    plt.axhline(0.0762, color='red', linestyle='--')
    plt.axhline(-0.0762, color='red', linestyle='--')
    plt.title("Y Position Error [m]")
    plt.savefig("SUPER_NICE_CODE/y_positional_error.png")
    plt.show()



make_3D_plot(["SUPER_NICE_CODE/fusion_log.csv"], 1, title="KF Position", file_name="SUPER_NICE_CODE/graph100.png")