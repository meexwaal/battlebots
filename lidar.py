import numpy as np
import matplotlib.pyplot as plt
from collections import deque

from common import *
from sensor import Sensor

class Lidar:
    "Collection of lidar-related functionality"

    def __init__(self):
        self.lidar_sensor = Sensor(
            sample_rate=1800,
            min_val=0.05,
            max_val=12,
            proportional_bias=0.01,
            noise_std=0.05,
        )

        # Half-length of arena walls
        self.bound = 1.2192 # meters, = 4'

        # Alg
        # Notes on gain:
        #  1 and 0.1 are ok, but once they get off they oscillate
        #  0.01 is too small to correct for 1% error in w
        self.kp = 0.1
        self.ki = 0.0
        self.kd = 0.01
        self.integrator = 0
        self.prev_feedback = 0
        self.est_w = 0
        self.est_theta = 0
        self.bad_est_count = 0
        self.avg_feedback = 0.01
        self.locked = False
        self.est_x = 0.0
        self.est_y = 0.0

        # Maintain a history of sensed points and the alignment of vectors
        hist_len = 20
        self.hist_points = deque(maxlen=hist_len)
        self.hist_aligns = deque(maxlen=hist_len)

    def get_curvatures(self, ps):
        """
        Calculate some measure of curvature for each consecutive triple of points.
        Returns an array of scalars with 2 fewer elements than the input,
        and a weight for each measurement.

        Curvature:
          >> 0    > 0       = 0       < 0     << 0
         ------ -------- --------- --------- ------
           C          C              A--B     A--B
            \        /    A--B--C        \      /
          A--B   A--B                     C    C

        """
        # Unit vector from one point to next
        vs_full = np.diff(ps, axis=0)
        vs = unit(vs_full)

        ab = vs[:-1]
        bc = vs[1:]

        # Weight based on lengths
        lengths = norm(vs_full)
        weights = lengths[:-1] + lengths[1:]

        ret = signed_angle(ab, bc)

        return ret, weights

    def get_alignments(self, ps):
        """
        Calculate some measure of how far from the nearest horizontal or vertical each
        segment is. Returns an array of scalars with 1 fewer element than the input,
        a weight for each measurement, and whether each segment is vertical
        """
        vs = np.diff(ps, axis=0)
        # Reference unit vectors, each along the axis that the vector is best aligned with
        is_vert = np.abs(vs[:,1]) > np.abs(vs[:,0])
        ref = np.sign(vs)
        ref[is_vert,0] = 0
        ref[~is_vert,1] = 0

        return signed_angle(vs, ref), norm(vs), is_vert


    def measure(self, truth_x, truth_y, truth_theta, dt):
        "Measure distance to a wall from a position inside of an 8' square"

        # Unit look vector
        vx = np.cos(truth_theta)
        vy = np.sin(truth_theta)

        # Avoid the edge case of 0. When a component of the look vector is zero, the
        # other component will be closer to a wall anyways, so a small increase in
        # the zero component doesn't matter.
        if vx == 0:
            vx = 1e-5
        if vy == 0:
            vy = 1e-5

        # Get the two walls that the look vector intersects with
        xbound = self.bound * np.sign(vx)
        ybound = self.bound * np.sign(vy)

        # Get the distance to the intersection with each wall
        dx = (xbound - truth_x) / vx
        dy = (ybound - truth_y) / vy

        # The closer wall occludes the further one
        truth_dist = min(dx, dy)

        return self.lidar_sensor.step(truth_dist, dt)

    def align_alg(self, truth_x, truth_y, truth_theta, dt, sense_w=None):
        "Estimates theta and w using lidar, trying to align edges to horizontal and vertical"

        # Measure distance at the true angle
        d = self.measure(truth_x, truth_y, truth_theta, dt)
        if d is None:
            return

        # Trust the sensed angular rate, but only within a few percent
        if sense_w is not None and abs(self.est_w - sense_w) > 0.05 * sense_w:
            self.bad_est_count += 1
        if self.bad_est_count > 100:
            print(f"Lidar resetting est_w ({self.est_w:.3f}) to match accel ({sense_w:.3f})")
            self.est_w = sense_w
            self.integrator = 0
            self.bad_est_count = 0

        # Calc point at the sensed angle
        sense_point_rel = np.array((
            d * np.cos(self.est_theta),
            d * np.sin(self.est_theta)))
        self.hist_points.append(
            np.array((self.est_x, self.est_y)) + sense_point_rel)

        if len(self.hist_points) >= 2:
            align, weight, is_vert = self.get_alignments(
                np.array((self.hist_points[-2], self.hist_points[-1])))
            self.hist_aligns.append(align[0])

            # TODO: what's best here? Do we even want to use the history? Adds weird extra filtering
            feedback = np.mean(self.hist_aligns)
            # feedback = np.median(self.hist_aligns)

            # Other ideas
            # np.average(self.hist_aligns, weights=weight)
            # weighted_median(self.hist_aligns, weights)
            #
            # s = np.sort(self.hist_aligns)
            # keep = 3
            # drop = (N - 2 - keep)//2
            # np.mean(s[drop:-drop])

            # Update position based on which wall we're pointing at, once the
            # angle loop has locked
            pos_alpha = 0.02
            if self.locked:
                if is_vert[0]:
                    if np.cos(self.est_theta) > 0:
                        measured_pos = self.bound - sense_point_rel[0]
                    else:
                        measured_pos = -self.bound - sense_point_rel[0]
                    self.est_x = alpha_filter(pos_alpha, self.est_x, measured_pos)
                else:
                    if np.sin(self.est_theta) > 0:
                        measured_pos = self.bound - sense_point_rel[1]
                    else:
                        measured_pos = -self.bound - sense_point_rel[1]
                    self.est_y = alpha_filter(pos_alpha, self.est_y, measured_pos)

        else:
            feedback = 0


        self.integrator += feedback * dt
        derivative = (feedback - self.prev_feedback) / dt
        self.prev_feedback = feedback

        # print(f"est_w: {self.est_w} "
        #       f"P: {self.kp * feedback:.5f} I: {self.ki * self.integrator:.5f} "
        #       f"D: {self.kd * derivative:.5f}")
        est_w_delta = self.kp * feedback + self.ki * self.integrator + self.kd * derivative
        self.est_w += est_w_delta

        self.avg_feedback = alpha_filter(0.002, self.avg_feedback, est_w_delta)
        # print(self.avg_feedback)
        if abs(self.est_w) > 200 and abs(self.avg_feedback) < 0.01 and not self.locked:
            self.locked = True
            print("Lidar locked")
        if abs(self.avg_feedback) > 0.05 and self.locked:
            self.locked = False
            print("Lidar unlocked")

        self.est_theta += self.est_w * dt
        self.est_theta = np.fmod(self.est_theta, 2 * np.pi)


def test(speed_scale=1, plot=True):
    N = 29

    lidar = Lidar()
    lidar.kp = 0
    lidar.ki = 0
    lidar.kd = 0
    lidar.est_w = 2 * np.pi / (N-1)

    true_angle = np.linspace(0, 2*np.pi, N) * speed_scale

    start_angle = np.random.rand() * np.pi * 2
    true_angle += start_angle


    # x = 1.1
    # y = 1.1

    x = -0.4
    y = -0.8

    for i in range(len(true_angle)):
        lidar.align_alg(x, y, true_angle[i], 1)

    if plot:
        fig, ax = plt.subplots()

        ax.set_xlim(-2 * lidar.bound, 2 * lidar.bound)
        ax.set_ylim(-2 * lidar.bound, 2 * lidar.bound)

        ax.plot((-lidar.bound, -lidar.bound, lidar.bound, lidar.bound, -lidar.bound),
                (-lidar.bound, lidar.bound, lidar.bound, -lidar.bound, -lidar.bound),
                color='black', linewidth=1)

        ps = np.array(lidar.hist_points) + np.array((x, y))
        ax.plot(ps[:,0], ps[:,1], 'o')
        ax.plot([x], [y], 'ro')
        plt.gca().set_aspect('equal')
        plt.show()

    return np.mean(lidar.hist_aligns)


if __name__ == '__main__':
    test(speed_scale=1., plot=True)

    for s in [0.9, 1, 1.1]:
        m = np.zeros(100)
        for i in range(len(m)):
            m[i] = test(speed_scale=s, plot=False)

        print()
        print(s)
        print('---')
        print(f"{np.mean(m):.3f}, {np.std(m):.3f}")
