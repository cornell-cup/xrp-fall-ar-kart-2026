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

april_tag_locations = [[1, -0.4572]]

# def wrap_pi(a):
#         return (a + np.pi) % (2.0 * np.pi) - np.pi

def get_theta(x1, y1, x2, y2):
    print(f"x1, y1: {x1}, {y1}")
    print(f"x2, y1: {x2}, {y2}")
    yaw = np.degrees(np.arctan2(y2-y1,x2-x1))
    print(f"yaw: {yaw}")
    return yaw

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


    ekf_x = []
    ekf_y = []
    ekf_theta = []
    ball_x = []
    ball_y = []
    ball_theta = []
    time = []
    april_tag_x = []
    april_tag_y = []

    ball_yaw_interval = 50

    for file in csv_files:
        with open(file) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line = 0
            for row in csv_reader:
                if line != 0:
                    time.append(float(row[0]))
                    ekf_x.append(float(row[1]))
                    ekf_y.append(float(row[2]))
                    ekf_theta.append(np.degrees(float(row[3])))
                    ball_x.append(float(row[7]))
                    ball_y.append(float(row[8]))
                    if line >= ball_yaw_interval:
                        ball_theta.append((get_theta(ball_x[-ball_yaw_interval], ball_y[-ball_yaw_interval], ball_x[-1], ball_y[-1])))
                    else:
                        ball_theta.append(ekf_theta[-1])

                    for i in range(len(april_tag_locations)):
                        april_tag_x.append(april_tag_locations[i][0])
                        april_tag_y.append(april_tag_locations[i][1])
                    line = line + 1
                else:
                    line = line + 1

    # # Create a figure and a 3D axes object
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # ax.scatter3D(ekf_x, ekf_y, time, c=time, cmap='Reds', marker='o', label='KF')
    # ax.scatter3D(ball_x, ball_y, time, c=time, cmap='Blues', marker='o', label='Ball')
    # ax.scatter3D(april_tag_x, april_tag_y, time, c=time)

    # # Set labels for the axes
    # ax.set_xlabel('x-axis')
    # ax.set_ylabel('y-axis')
    # ax.set_zlabel('t-axis')

    # # Set a title for the plot
    # ax.set_title(title)

    # # Display the plot
    # plt.legend()
    # plt.savefig("3Dpath.png")
    # plt.show()

    ekf_x = np.array(ekf_x)
    ekf_y = np.array(ekf_y)
    ekf_theta = np.array(ekf_theta)

    ball_x = np.array(ball_x)
    ball_y = np.array(ball_y)
    ball_theta = np.array(ball_theta)

    error_x = ball_x - ekf_x
    error_y = ball_y - ekf_y
    error_theta = ball_theta - ekf_theta

    # plt.plot(time, error_x)
    # plt.axhline(0.0762, color='red', linestyle='--')
    # plt.axhline(-0.0762, color='red', linestyle='--')
    # plt.title("X Position Error [m]")
    # plt.savefig("x_positional_error.png")
    # plt.show()

    # plt.plot(time, error_y)
    # plt.axhline(0.0762, color='red', linestyle='--')
    # plt.axhline(-0.0762, color='red', linestyle='--')
    # plt.title("Y Position Error [m]")
    # plt.savefig("y_positional_error.png")
    # plt.show()

    # plt.plot(time, ball_theta)
    plt.plot(time, error_theta)
    # plt.axhline(0.0762, color='red', linestyle='--')
    # plt.axhline(-0.0762, color='red', linestyle='--')
    plt.title("Error in Yaw [degrees]")
    plt.savefig("yaw_error.png")
    plt.show()



make_3D_plot(["fusion_log.csv"], 1, title="KF Position", file_name="graph100.png")