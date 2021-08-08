# Group26_Python-Inverse_Kinematics.py
# Python kinematics implementation and demo.

import matplotlib.pyplot as plt
import numpy as np


# Cheetah-Bird Design
Kp = 15
dt = 0.01

l1 = 0.8
l2 = 0.6

# initial position of EE (foot)
x = 0
y = -0.8

show_animation = True

if show_animation:
    plt.ion()


def two_rev_leg(target_pos=0.0, q1=0.0, q2=0.0):
    """
    Inverse Dynamic Demo
    """
    global x, y
    while True:
        try:
            if x is not None and y is not None:
                x_prev = x
                y_prev = y
            if np.sqrt(x**2 + y**2) > (l1 + l2):
                q2_tgt = 0
            else:
                q2_tgt = np.arccos(
                    (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
            q1_tgt = np.math.atan2(y, x) - np.math.atan2(l2 *
                                                              np.sin(q2_tgt), (l1 + l2 * np.cos(q2_tgt)))

            if q1_tgt < 0:
                q2_tgt = -q2_tgt
                q1_tgt = np.math.atan2(
                    y, x) - np.math.atan2(l2 * np.sin(q2_tgt), (l1 + l2 * np.cos(q2_tgt)))

            q1 = q1 + Kp * ang_diff(q1_tgt, q1) * dt
            q2 = q2 + Kp * ang_diff(q2_tgt, q2) * dt
        except ValueError as e:
            print("Cannot reach the desired position")
        except TypeError:
            x = x_prev
            y = y_prev

        ankle = plot_arm(q1, q2, x, y)


        if x is not None and y is not None:
            d2goal = np.hypot(ankle[0] - x, ankle[1] - y)

        if abs(d2goal) < target_pos and x is not None:
            return q1, q2


def plot_arm(q1, q2, x, y):  # pragma: no cover
    hip = np.array([0, 0])
    knee = hip + np.array([l1 * np.cos(q1), l1 * np.sin(q1)])
    ankle = knee + np.array([l2 * np.cos(q1 + q2), l2 * np.sin(q1 + q2)])

    if show_animation:
        plt.cla()

        plt.plot([hip[0], knee[0]], [hip[1], knee[1]], color='#1e90ff', linestyle='-',lw=3)
        plt.plot([knee[0], ankle[0]], [knee[1], ankle[1]], color='#1e90ff', linestyle='-',lw=3)

        plt.plot(hip[0], hip[1], color='#3d4b4d',marker='o',ms=10)
        plt.plot(knee[0], knee[1], color='#3d4b4d',marker='o',ms=10)
        plt.plot(ankle[0], ankle[1], color='#3d4b4d',marker='^',ms=15)

        plt.plot([ankle[0], x], [ankle[1], y], 'k--')
        plt.plot(x, y, 'k*')

        plt.xlim(-2, 2)
        plt.ylim(-2, 2)

        plt.show()
        plt.pause(dt)

    return ankle


def ang_diff(q1, q2):
    # Returns the difference between two angles in the range -pi to +pi
    return (q1 - q2 + np.pi) % (2 * np.pi) - np.pi


def click(event):  # pragma: no cover
    global x, y
    x = event.xdata
    y = event.ydata


def animation():
    from random import random
    global x, y
    q1 = q2 = 0.0
    for i in range(5):
        x = 2.0 * random() - 1.0
        y = 2.0 * random() - 1.0
        q1, q2 = two_rev_leg(
            GOAL_TH=0.01, q1=q1, q2=q2)


def main():  # pragma: no cover
    fig = plt.figure()
    fig.canvas.mpl_connect("button_press_event", click)
    # for stopping simulation with the esc key.
    fig.canvas.mpl_connect('key_release_event', lambda event: [
                           exit(0) if event.key == 'escape' else None])
    two_rev_leg(q1=-0.7)


if __name__ == "__main__":
    # animation()
    main()
