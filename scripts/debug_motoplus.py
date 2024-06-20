import numpy as np

def find_max_vel(filename):
    with open(filename, 'r') as f:
        lines = f.read().split("\n")

        robot_a_vels = []
        robot_b_vels = []
        for line in lines[17:]:
            if line.startswith("Vel A"):
                vels = line[7:].split()
                vels = [float(vel) for vel in vels]
                robot_a_vels.append(vels)
            if line.startswith("Vel B"):
                vels = line[7:].split()
                vels = [float(vel) for vel in vels]
                robot_b_vels.append(vels)
        
        robot_a_vels = np.array(robot_a_vels)
        robot_b_vels = np.array(robot_b_vels)

        robot_a_acc = robot_a_vels[:, 1:] - robot_a_vels[:, :-1]
        robot_b_acc = robot_b_vels[:, 1:] - robot_b_vels[:, :-1]
        print(robot_a_vels.min(axis=0), robot_a_vels.max(axis=0))
        print(robot_b_vels.min(axis=0), robot_b_vels.max(axis=0))
        print("Acceleration:")
        print(robot_a_acc.min(axis=0), robot_a_acc.max(axis=0))
        print(robot_b_acc.min(axis=0), robot_b_acc.max(axis=0))

if __name__ == "__main__":
    filename = "/home/mfi/repos/ros1_ws/output.txt"
    find_max_vel(filename)