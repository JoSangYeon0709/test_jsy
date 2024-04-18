import tkinter as tk
from tkinter import ttk
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

PLOT_AREA = 0.5

class Link:
    def __init__(self, dh_params):
        self.dh_params_ = dh_params

    def transformation_matrix(self):
        theta = self.dh_params_[0]
        alpha = self.dh_params_[1]
        a = self.dh_params_[2]
        d = self.dh_params_[3]

        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        trans = np.array([[ct, -st * ca, st * sa, a * ct],
                          [st, ct * ca, -ct * sa, a * st],
                          [0, sa, ca, d],
                          [0, 0, 0, 1]])
        return trans

    @staticmethod
    def basic_jacobian(trans_prev, ee_pos):
        pos_prev = np.array(
            [trans_prev[0, 3], trans_prev[1, 3], trans_prev[2, 3]])
        z_axis_prev = np.array(
            [trans_prev[0, 2], trans_prev[1, 2], trans_prev[2, 2]])

        basic_jacobian = np.hstack(
            (np.cross(z_axis_prev, ee_pos - pos_prev), z_axis_prev))
        return basic_jacobian


class NLinkArm:
    def __init__(self, dh_params_list):
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))

        # Initialize self.ax attribute
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.plot()

    @staticmethod
    def convert_joint_angles_sim_to_mycobot(joint_angles):
        conv_mul = [-1.0, -1.0, 1.0, -1.0, -1.0, -1.0]
        conv_add = [0.0, -math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, 0.0]

        joint_angles = [joint_angles * conv_mul for (joint_angles, conv_mul) in zip(joint_angles, conv_mul)]
        joint_angles = [joint_angles + conv_add for (joint_angles, conv_add) in zip(joint_angles, conv_add)]

        joint_angles_lim = []
        for joint_angle in joint_angles:
            while joint_angle > math.pi:
                joint_angle -= 2 * math.pi

            while joint_angle < -math.pi:
                joint_angle += 2 * math.pi

            joint_angles_lim.append(joint_angle)

        return joint_angles_lim

    def transformation_matrix(self):
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
        return trans

    def forward_kinematics(self):
        trans = self.transformation_matrix()

        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        alpha, beta, gamma = self.euler_angle()

        return [x, y, z, alpha, beta, gamma]

    def basic_jacobian(self):
        ee_pos = self.forward_kinematics()[0:3]
        basic_jacobian_mat = []

        trans = np.identity(4)
        for i in range(len(self.link_list)):
            basic_jacobian_mat.append(
                self.link_list[i].basic_jacobian(trans, ee_pos))
            trans = np.dot(trans, self.link_list[i].transformation_matrix())

        return np.array(basic_jacobian_mat).T

    def inverse_kinematics(self, ref_ee_pose):
        for cnt in range(500):
            ee_pose = self.forward_kinematics()
            diff_pose = np.array(ref_ee_pose) - ee_pose

            basic_jacobian_mat = self.basic_jacobian()
            alpha, beta, gamma = self.euler_angle()

            K_zyz = np.array(
                [[0, -math.sin(alpha), math.cos(alpha) * math.sin(beta)],
                 [0, math.cos(alpha), math.sin(alpha) * math.sin(beta)],
                 [1, 0, math.cos(beta)]])
            K_alpha = np.identity(6)
            K_alpha[3:, 3:] = K_zyz

            theta_dot = np.dot(
                np.dot(np.linalg.pinv(basic_jacobian_mat), K_alpha),
                np.array(diff_pose))
            self.update_joint_angles(theta_dot / 100.)

        self.plot()

    def euler_angle(self):
        trans = self.transformation_matrix()

        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi
        beta = math.atan2(
            trans[0][2] * math.cos(alpha) + trans[1][2] * math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0] * math.sin(alpha) + trans[1][0] * math.cos(alpha),
            -trans[0][1] * math.sin(alpha) + trans[1][1] * math.cos(alpha))

        return alpha, beta, gamma

    def send_angles(self, joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] = joint_angle_list[i]

    def update_joint_angles(self, diff_joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] += diff_joint_angle_list[i]

    def plot(self):
        self.ax.cla()  # Clear previous plot
        x_list = []
        y_list = []
        z_list = []

        trans = np.identity(4)

        x_list.append(trans[0, 3])
        y_list.append(trans[1, 3])
        z_list.append(trans[2, 3])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])

        # Plot the arm links
        self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4, mew=0.5)
        # Plot the end effector point in orange
        self.ax.plot([x_list[-1]], [y_list[-1]], [z_list[-1]], "o", color="orange")

        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

        self.ax.set_xlim(-PLOT_AREA, PLOT_AREA)
        self.ax.set_ylim(-PLOT_AREA, PLOT_AREA)
        self.ax.set_zlim(-PLOT_AREA, PLOT_AREA)

        plt.draw()
        plt.pause(0.001)


if __name__ == "__main__":
    # Tkinter 애플리케이션 생성
    root = tk.Tk()
    root.title('Arm Control')

    def update_plot():
        x_axis_val = float(x_axis_entry.get())
        y_axis_val = float(y_axis_entry.get())
        z_axis_val = float(z_axis_entry.get())
        x_degree_val = float(x_degree_entry.get())
        y_degree_val = float(y_degree_entry.get())
        z_degree_val = float(z_degree_entry.get())

        x_degree_val_rad = math.radians(x_degree_val)
        y_degree_val_rad = math.radians(y_degree_val)
        z_degree_val_rad = math.radians(z_degree_val)

        mycobot_sim.inverse_kinematics([x_axis_val, y_axis_val, z_axis_val,
                                         x_degree_val_rad, y_degree_val_rad, z_degree_val_rad])

    # X, Y, Z 입력 상자 및 라벨 생성
    x_axis_frame = ttk.Frame(root)
    x_axis_frame.grid(row=0, column=0, padx=5, pady=5)
    x_axis_label = ttk.Label(x_axis_frame, text='X:')
    x_axis_label.grid(row=0, column=0, padx=5, pady=5)
    x_axis_entry = ttk.Entry(x_axis_frame, width=10)
    x_axis_entry.grid(row=1, column=0, padx=5, pady=5)
    x_axis_entry.insert(0, '0.0')

    y_axis_frame = ttk.Frame(root)
    y_axis_frame.grid(row=0, column=1, padx=5, pady=5)
    y_axis_label = ttk.Label(y_axis_frame, text='Y:')
    y_axis_label.grid(row=0, column=0, padx=5, pady=5)
    y_axis_entry = ttk.Entry(y_axis_frame, width=10)
    y_axis_entry.grid(row=1, column=0, padx=5, pady=5)
    y_axis_entry.insert(0, '0.0')

    z_axis_frame = ttk.Frame(root)
    z_axis_frame.grid(row=0, column=2, padx=5, pady=5)
    z_axis_label = ttk.Label(z_axis_frame, text='Z:')
    z_axis_label.grid(row=0, column=0, padx=5, pady=5)
    z_axis_entry = ttk.Entry(z_axis_frame, width=10)
    z_axis_entry.grid(row=1, column=0, padx=5, pady=5)
    z_axis_entry.insert(0, '0.0')

    x_degree_frame = ttk.Frame(root)
    x_degree_frame.grid(row=1, column=0, padx=5, pady=5)
    x_degree_label = ttk.Label(x_degree_frame, text='X degree:')
    x_degree_label.grid(row=1, column=0, padx=5, pady=5)
    x_degree_entry = ttk.Entry(x_degree_frame, width=10)
    x_degree_entry.grid(row=2, column=0, padx=5, pady=5)
    x_degree_entry.insert(0, '0.0')

    y_degree_frame = ttk.Frame(root)
    y_degree_frame.grid(row=1, column=1, padx=5, pady=5)
    y_degree_label = ttk.Label(y_degree_frame, text='Y degree:')
    y_degree_label.grid(row=1, column=0, padx=5, pady=5)
    y_degree_entry = ttk.Entry(y_degree_frame, width=10)
    y_degree_entry.grid(row=2, column=0, padx=5, pady=5)
    y_degree_entry.insert(0, '0.0')

    z_degree_frame = ttk.Frame(root)
    z_degree_frame.grid(row=1, column=2, padx=5, pady=5)
    z_degree_label = ttk.Label(z_degree_frame, text='Z degree:')
    z_degree_label.grid(row=1, column=0, padx=5, pady=5)
    z_degree_entry = ttk.Entry(z_degree_frame, width=10)
    z_degree_entry.grid(row=2, column=0, padx=5, pady=5)
    z_degree_entry.insert(0, '0.0')

    # 그래프 업데이트 버튼 생성
    update_button = ttk.Button(root, text='Update Plot', command=update_plot)
    update_button.grid(row=1, column=3, padx=5, pady=5)

    # 초기 그래프 표시
    mycobot_sim = NLinkArm([[0., math.pi / 2, 0, 0.13156],
                            [0., 0., -0.1104, 0.],
                            [0., 0., -0.096, 0.],
                            [0., math.pi / 2, 0., 0.06639],
                            [0., -math.pi / 2, 0., 0.07318],
                            [0., 0., 0., 0.0436]])

    mycobot_sim.send_angles(
        mycobot_sim.convert_joint_angles_sim_to_mycobot
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    # Update the plot after sending initial joint angles
    mycobot_sim.plot()

    # Tkinter main loop 시작
    root.mainloop()
