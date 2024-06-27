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
        and a weight for each measurement.
        """
        vs = np.diff(ps, axis=0)
        # Reference unit vectors, each along the axis that the vector is best aligned with
        is_vert = np.abs(vs[:,1]) > np.abs(vs[:,0])
        ref = np.sign(vs)
        ref[is_vert,0] = 0
        ref[~is_vert,1] = 0

        return signed_angle(vs, ref), norm(vs)


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

    def align_alg(self, truth_x, truth_y, truth_theta, dt):
        "Estimates theta and w using lidar, trying to align edges to horizontal and vertical"

        # todo:
        # incorporate external w estimate, e.g. from accelerometer
        # incorporate position estimate

        # Measure distance at the true angle
        d = self.measure(truth_x, truth_y, truth_theta, dt)
        if d is None:
            return

        # Calc point at the sensed angle
        # TODO: add position estimate here
        sense_point = np.array((
            d * np.cos(self.est_theta),
            d * np.sin(self.est_theta)))
        self.hist_points.append(sense_point)

        if len(self.hist_points) >= 2:
            align, weight = self.get_alignments(
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

        else:
            feedback = 0


        self.integrator += feedback * dt
        derivative = (feedback - self.prev_feedback) / dt
        self.prev_feedback = feedback

        #print(f"P: {self.kp * feedback:.5f} I: {self.ki * self.integrator:.5f} "
        #      f"D: {self.kd * derivative:.5f}")
        self.est_w += self.kp * feedback + self.ki * self.integrator + self.kd * derivative

        self.est_theta += self.est_w * dt
        self.est_theta = np.fmod(self.est_theta, 2 * np.pi)


        # todo: hack because this all breaks if est_w is really bad
        if abs(self.est_w) < 1:
            self.est_w = np.random.choice([-1,1])

    # TODO: calculate position
    #   option 1: just measure it
    #     you can pretty easily pick out the points on each wall, using min/max or horizontal/vert
    #     do the geometry, you get a bunch of readings of the distance to wall
    #     just do a weighted mean or median or something
    #   option 2: hypothesize positions
    #     create readings from hypothesized positions
    #     pick the hypothesis that most closely matches
    #     can zoom in and repeat for more accuracy


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
