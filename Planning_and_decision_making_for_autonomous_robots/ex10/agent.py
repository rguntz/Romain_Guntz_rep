import random
from dataclasses import dataclass
from re import X
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from typing import Sequence

from commonroad.scenario.lanelet import LaneletNetwork
from dg_commons import PlayerName
from dg_commons.sim.goals import PlanningGoal
from dg_commons.sim import SimObservations, InitSimObservations
from dg_commons.sim.agents import Agent
from dg_commons.sim.models.obstacles import StaticObstacle
from dg_commons.sim.models.vehicle import VehicleCommands
from dg_commons.sim.models.vehicle_structures import VehicleGeometry
from dg_commons.sim.models.vehicle_utils import VehicleParameters
import numpy as np
import math
import bisect
import copy
from shapely.affinity import translate, rotate, scale
from shapely.geometry import Polygon, LineString
import os
from matplotlib.cm import get_cmap
from sympy import print_ccode

# Parameter
MAX_SPEED = 35  # maximum speed [m/s]
MAX_ACCEL = 5.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7  # maximum road width [m]
D_ROAD_W = 3  # road width sampling length [m]
DT = 0.5  # time tick [s]
MAXT = 6.0  # max prediction time [m]
MINT = 4.0  # min prediction time [m]
TARGET_SPEED = 25  # target speed [m/s]
D_T_S = 3  # target speed sampling length [m/s]
N_S_SAMPLE = 3  # sampling number of target speed
ROBOT_RADIUS = 2.0  # robot radius [m]
TARGET_SPEED_MIN = 0
TARGET_SPEED_ORIGINAL = 25

ORIGINAL_D_ROAD_W = 3
ORIGINAL_N_S_SAMPLE = 3
CROWDED = False

END_OF_TRACK = None
TRACK_HAS_ENDED = False

# cost weights
KJ = 0.1
KT = 1
KD = 1.0
KLAT = 16
KLON = 1.0


@dataclass(frozen=True)
class Pdm4arAgentParams:
    param1: float = 0.2
    planning_dt: float = 1.0  # Re-plan every 1 second
    target_speed: float = 30.0 / 3.6
    max_accel: float = 2.0
    max_speed: float = 50.0 / 3.6
    robot_radius: float = 3.0


class Pdm4arAgent(Agent):
    """This is the PDM4AR agent implementing a path planning and control stack using quintic polynomials"""

    name: PlayerName
    goal: PlanningGoal
    sg: VehicleGeometry
    sp: VehicleParameters

    def __init__(self):
        self.params = Pdm4arAgentParams()
        self.planning_timer = 0.0
        self.current_trajectory = None
        self.trajectory_t0 = 0.0
        self.last_plan_time = -999.0
        self.c_d_d = 0
        self.c_d_dd = 0
        self.done_episode_init = False
        self.yaw_error_last = 0.0
        self.yaw_error_integral = 0.0
        self.total_time = 0

    def on_episode_init(self, init_obs: InitSimObservations):
        self.index_next_collision = 0
        self.init_obs = init_obs

        """Initialize the agent: extract lane info, setup spline, etc."""
        self.name = init_obs.my_name
        self.goal = init_obs.goal
        self.sg = init_obs.model_geometry
        self.sp = init_obs.model_params

        # Extract the centerline of the desired lane from the goal:
        ref_lane = self.goal.ref_lane
        control_points = ref_lane.control_points
        # Convert these control points to arrays
        wx = [cp.q.p[0] for cp in control_points]
        wy = [cp.q.p[1] for cp in control_points]

        x1 = wx[-2]
        x2 = wx[-1]
        y1 = wy[-2]
        y2 = wy[-1]

        m = (y2 - y1) / (x2 - x1)
        p = y2 - m * x2

        x = wx[-1] + (wx[-1] - wx[-2]) * 2
        y = m * x + p

        wx.append(x)
        wy.append(y)

        self.static_line_obstacles = []
        for static_obs in init_obs.dg_scenario.static_obstacles:
            # The shape property is a CommonRoad geometry object; shapely_object converts it to a Shapely geometry
            shapely_line = static_obs.shape
            self.static_line_obstacles.append(shapely_line)

    def on_episode_init_2(self, init_obs: InitSimObservations, x_sup, y_sup):
        """Initialize the agent: extract lane info, setup spline, etc."""
        self.name = init_obs.my_name
        self.goal = init_obs.goal
        self.sg = init_obs.model_geometry
        self.sp = init_obs.model_params

        # Extract the centerline of the desired lane from the goal:
        ref_lane = self.goal.ref_lane
        control_points = ref_lane.control_points
        # Convert these control points to arrays
        wx = [cp.q.p[0] for cp in control_points]
        wy = [cp.q.p[1] for cp in control_points]

        wx = wx[-2:]
        wy = wy[-2:]

        x1 = wx[-2]
        x2 = wx[-1]
        y1 = wy[-2]
        y2 = wy[-1]

        m = (y2 - y1) / (x2 - x1)
        p = y2 - m * x2

        x = wx[-1] + (wx[-1] - wx[-2]) * 2
        y = m * x + p

        global END_OF_TRACK
        END_OF_TRACK = (x, y)

        sign_variable = 0
        self.sense = 0

        if x_sup < wx[-2]:
            sign_variable = -1
            self.sense = +1
        else:  # (x1 > wx[-2])
            sign_variable = +1
            self.sense = -1

        x1 = x_sup + sign_variable * 11
        y1 = m * x1 + p

        wx.append(x)
        wy.append(y)

        wx = np.insert(wx, 0, x1)
        wy = np.insert(wy, 0, y1)

        # Create a spline from these waypoints
        generate_target_course = generate_target_course_computation()
        rx_test, ry_test, ryaw_test, rk_test, csp = generate_target_course.generate_target_course(wx, wy)
        self.csp = csp
        self.rx = rx_test
        self.ry = ry_test

        plt.plot(rx_test, ry_test)

        self.plot = plt

        self.done_episode_init = True

    def get_obs_speed(self, sim_obs: SimObservations):
        obstacle_data = {}  # Dictionary to hold pname and (vx, x) pairs
        player_names = [pname for pname in sim_obs.players if pname != self.name]  # Exclude self player name

        for pname in player_names:
            pdata = sim_obs.players[pname]  # Retrieve player data
            new_vx = pdata.state.vx  # Get velocity in the x direction
            x_coord = pdata.state.x  # Get the x-coordinate
            obstacle_data[pname] = (new_vx, x_coord)  # Map pname to the (vx, x) tuple

        return obstacle_data

    def compute_obstacles(self, sim_obs: SimObservations):
        # Extract other vehicles' positions as obstacles
        # Visible vehicles are in sim_obs.players. Our own name is self.name
        obstacles = []
        player_names = [pname for pname in sim_obs.players if pname != self.name]
        num_obstacles = len(player_names)

        if num_obstacles > 0:
            # Create a color map
            color_map = cm.get_cmap("hsv", num_obstacles)

            for idx, pname in enumerate(player_names):
                pdata = sim_obs.players[pname]
                # Vehicle position
                vx = pdata.state.x
                vy = pdata.state.y
                obstacles.append([vx, vy])

                # Assign a unique color from the color map
                color = color_map(idx)
                self.plot.scatter(vx, vy, color=color, marker="x", label=pname)

        if len(obstacles) == 0:
            return np.empty((0, 2))
        return np.array(obstacles)

    def compute_obstacles_2(self, sim_obs: SimObservations):
        # Extract other vehicles' positions as obstacles
        # Visible vehicles are in sim_obs.players. Our own name is self.name
        obstacles = []
        speed = []
        player_names = [pname for pname in sim_obs.players if pname != self.name]
        num_obstacles = len(player_names)

        if num_obstacles > 0:
            # Create a color map
            color_map = cm.get_cmap("hsv", num_obstacles)

            for idx, pname in enumerate(player_names):
                pdata = sim_obs.players[pname]
                # Vehicle position
                vx = pdata.state.x
                vy = pdata.state.y
                obstacles.append([vx, vy])
                speed.append(pdata.state.vx)

        if len(obstacles) == 0:
            return np.empty((0, 2)), np.empty((0, 1))

        return np.array(obstacles), speed

    def plan_trajectory(self, static_line_obstacles, sim_obs: SimObservations, index_next_collision, time_since_simul):
        """Plan a trajectory using the frenet optimal planning approach."""

        # Current state from sim
        ego_state = sim_obs.players[self.name].state
        x, y, yaw, v, delta = ego_state.x, ego_state.y, ego_state.psi, ego_state.vx, ego_state.delta

        if self.done_episode_init is False:
            self.on_episode_init_2(self.init_obs, x, y)

        # Convert current pose to Frenet frame (approximately)
        # We'll find the closest point on the spline:
        s0, lateral_distance, slope = self.csp.find_frenet(x, y, self.rx, self.ry, self.sense)
        self.slope = slope
        # print("lateral and s are:", lateral_distance, s0)
        c_speed = v
        c_d = lateral_distance

        # Approximate lateral derivatives (assuming small changes):
        # A more robust approach would track previous states.

        ob = self.compute_obstacles(sim_obs)
        # Use the frenet_optimal_planning code (adapted from provided snippet)
        frenet_optimal_planning = frenet_optimal_planning_computation()
        best_path, all_path, prediction_test = frenet_optimal_planning.frenet_optimal_planning(
            static_line_obstacles,
            self.csp,
            s0,
            c_speed,
            c_d,
            self.c_d_d,
            self.c_d_dd,
            ob,
            sim_obs,
            self.name,
            self.sense,
            slope,
            index_next_collision,
            time_since_simul,
        )

        if best_path is not None:
            self.c_d_d = best_path.d_d[1]
            self.c_d_dd = best_path.d_dd[1]
            self.plot.plot(best_path.x, best_path.y)
            self.plot.scatter(x, y)

            for path_plot in all_path:
                self.plot.plot(path_plot.x, path_plot.y, alpha=0.05)
                pass

            # print("the speed information is :", best_path.s_d)

            for i in range(len(prediction_test)):
                # Extract x and y values from the array of tuples
                x_values = [point[0] for point in prediction_test[i]]
                y_values = [point[1] for point in prediction_test[i]]
                # Plot the points
                # self.plot.plot(x_values, y_values, alpha=0.6)  # Use marker='o' to show individual points

        if best_path is None:
            print("there is a None best_path, so adjusting to front car that is :", 0.0, "\n")
            fpplist = []
            # pdb.set_trace()

            ## Adjusted for the code :
            global_path_computer = calc_global_paths_computation()

            c_d_d = self.c_d_d
            c_d_dd = self.c_d_dd

            frenet_paths = []
            di = 0
            # Lateral motion planning
            for Ti in np.arange(0.5, 1.5, 0.1):
                fp = Frenet_path()

                lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                fp.t = [t for t in np.arange(0.0, Ti, DT)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Loongitudinal motion planning (Velocity keeping)
                tv = 0.0
                tfp = copy.deepcopy(fp)
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED_ORIGINAL - tfp.s_d[-1]) ** 2

                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1] ** 2
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv
                tfp.Ti = Ti

                if len(tfp.s) >= 3:
                    frenet_paths.append(tfp)
            fplist = frenet_paths

            fplist = global_path_computer.calc_global_paths(fplist, self.csp)

            # find minimum cost path
            mincost = float("inf")
            bestpath = None
            for fp in fplist:
                if mincost >= fp.cf:
                    mincost = fp.cf
                    bestpath = fp

            print("the best path speed is equal to :", bestpath.s_d)

            return bestpath

        return best_path

    def PID_Controller(self, path, sim_obs, x, y, yaw, v, delta):

        yaw_Kp = 1
        yaw_Ki = 0.5
        yaw_Kd = 1.8

        dt = 0.1
        current_time = sim_obs.time

        # idx = min(len(path.x) - 1, 5)  # next step in path
        idx = int(float(self.total_time) * self.increment_factor)
        idx = min(idx, len(path.y) - 2)

        # target heading
        target_yaw = math.atan2(path.y[idx + 1] - path.y[idx], path.x[idx + 1] - path.x[idx])
        # target speed
        target_v = path.s_d[idx + 1]
        # print("the target speed for the path is : ", target_v, "actual speed :", v)
        # print("total path is equal to :", path.s_d)
        # print("actual speed", v, "target", target_v)

        # Simple controller: Pure pursuit for steering, PID for speed
        # Steering control:
        # Compute desired heading error:
        yaw_error = target_yaw - yaw

        # Normalize angle:
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        yaw_error_derivative = (yaw_error - self.yaw_error_last) / dt
        self.yaw_error_integral += yaw_error * dt
        # Steering rate command:
        # For simplicity, directly set ddelta to reduce heading error
        # More sophisticated: A proper controller that respects steering limits
        ddelta_cmd = yaw_Kp * yaw_error + yaw_Ki * self.yaw_error_integral + yaw_Kd * yaw_error_derivative

        # Store the error for the next iteration
        self.yaw_error_last = yaw_error

        # Speed control:
        v_error = target_v - v
        k_speed = 20
        acc_cmd = k_speed * v_error
        # Clip acceleration:

        acc_cmd = np.clip(acc_cmd, self.sp.acc_limits[0], self.sp.acc_limits[1])
        ddelta_cmd = np.clip(ddelta_cmd, -self.sp.ddelta_max, self.sp.ddelta_max)

        print("time is :", current_time)

        self.plot.savefig("image.png")

        return acc_cmd, ddelta_cmd

    def compute_obstacles_ahead(self, sim_obs, x):

        number_of_obstacles = 0
        speed_mean = 0

        ob, speeds = self.compute_obstacles_2(sim_obs)

        for i, obstacles in enumerate(ob):  # 'i' is the index, 'obstacles' is the current element
            x_ob = obstacles[0]

            if self.sense > 0 and x_ob > x:
                number_of_obstacles += 1
                speed_mean += speeds[i]

            elif self.sense < 0 and x_ob < x:
                number_of_obstacles += 1
                speed_mean += speeds[i]

        if number_of_obstacles != 0:
            return number_of_obstacles, speed_mean / number_of_obstacles
        else:
            return number_of_obstacles, 0

    def get_commands(self, sim_obs: SimObservations) -> VehicleCommands:
        """Called at each step. We follow the planned trajectory.
        We re-plan at a lower frequency to reduce computation costs."""

        global CROWDED, MINT, MAXT, MAX_ROAD_WIDTH, TARGET_SPEED_ORIGINAL

        current_time = sim_obs.time

        print("the current time is :", current_time)

        # print("the current time is :", current_time)
        # if current_time == 1:
        #     #print("sims obs :", sim_obs)

        ego_state = sim_obs.players[self.name].state
        x, y, yaw, v, delta = ego_state.x, ego_state.y, ego_state.psi, ego_state.vx, ego_state.delta

        if self.done_episode_init is False:
            self.on_episode_init_2(self.init_obs, x, y)

        delta_x_end = END_OF_TRACK[0] - x
        delta_y_end = END_OF_TRACK[1] - y

        vector = np.array([delta_x_end, delta_y_end])
        norm = np.linalg.norm(vector)
        print("the norm is equal to :", norm)

        global TRACK_HAS_ENDED
        if norm < 10 or TRACK_HAS_ENDED is True:
            TRACK_HAS_ENDED = True
            return VehicleCommands(acc=0, ddelta=0)

        number_obstacles_ahead, speed_mean = self.compute_obstacles_ahead(sim_obs, x)
        print("the mean speed is equal to :", speed_mean)

        if speed_mean <= 12 and number_obstacles_ahead != 0:
            CROWDED = True
            MINT = 2.0
            MAXT = 3.0
            TARGET_SPEED_ORIGINAL = speed_mean
            print("CROWDED", "number of vehicle is :", number_obstacles_ahead)
        elif speed_mean > 12 or number_obstacles_ahead == 0:
            CROWDED = False
            MINT = 4.0
            MAXT = 6.0
            TARGET_SPEED_ORIGINAL = speed_mean
            print("NOT CROWDED", "number of vehicle is :", number_obstacles_ahead)

        self.current_trajectory = self.plan_trajectory(
            self.static_line_obstacles, sim_obs, self.index_next_collision, self.total_time
        )
        self.increment_factor = len(self.current_trajectory.s) / (self.current_trajectory.Ti)
        self.trajectory_t0 = current_time
        self.last_plan_time = current_time

        acc_cmd, ddelta_cmd = self.PID_Controller(self.current_trajectory, sim_obs, x, y, yaw, v, delta)

        return VehicleCommands(acc=acc_cmd, ddelta=ddelta_cmd)


class CubicSpline1D:
    def __init__(self, x, y):

        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        """
        Calc `y` position for given `x`.

        if `x` is outside the data point's `x` range, return None.

        Returns
        -------
        y : float
            y position for given x.
        """
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + self.c[i] * dx**2.0 + self.d[i] * dx**3.0

        return position

    def calc_first_derivative(self, x):
        """
        Calc first derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        dy : float
            first derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx**2.0
        return dy

    def calc_second_derivative(self, x):
        """
        Calc second derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        ddy : float
            second derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        x : float
            x position for given s.
        y : float
            y position for given s.
        """
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        k : float
            curvature for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx**2 + dy**2) ** (3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        yaw : float
            yaw angle (tangent vector) for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw

    def find_frenet(self, x, y, rx, ry, sense):
        """
        Find the Frenet coordinates (s0, lateral_error) for a given point (x, y).

        Parameters
        ----------
        x : float
            x position of the point.
        y : float
            y position of the point.

        csv : list of tuples
            Frenet path consisting of (x, y) tuples representing the reference path.

        Returns
        -------
        s0 : float
            Longitudinal distance along the spline from the start.
        lateral_error : float
            Lateral distance (perpendicular) from the point to the spline.
        """

        # Convert the csv path to a numpy array for easier manipulation
        path = np.vstack((rx, ry)).T

        # Calculate the pairwise distances between the point and all path points
        distances = np.linalg.norm(path - np.array([x, y]), axis=1)

        # Find the closest point on the path
        closest_index = np.argmin(distances)
        closest_point = path[closest_index]

        # Calculate lateral error (perpendicular distance)
        lateral_error = distances[closest_index]

        # Calculate s0 (longitudinal distance along the path)
        s0 = 0
        for i in range(closest_index):
            s0 += np.linalg.norm(path[i] - path[i + 1])

        ## Find the sign of d :
        slope = (ry[-1] - ry[0]) / (rx[-1] - rx[0])
        p = ry[-1] - slope * rx[-1]

        if sense > 0:
            if y < slope * x + p:
                lateral_error = -lateral_error
        else:  # (sense < 0)
            if y > slope * x + p:
                lateral_error = -lateral_error

        return s0, lateral_error, slope


class quintic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5], [3 * T**2, 4 * T**3, 5 * T**4], [6 * T, 12 * T**2, 20 * T**3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2, vxe - self.a1 - 2 * self.a2 * T, axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T**2, 4 * T**3], [6 * T, 12 * T**2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T, axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class Frenet_path:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []
        self.Ti = []


class calc_frenet_paths_computation:

    def calc_frenet_paths(self, c_speed, c_d, c_d_d, c_d_dd, s0):

        frenet_paths = []

        # generate path to each offset goal
        for di in np.linspace(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

            # Lateral motion planning
            for Ti in np.arange(MINT, MAXT, DT):
                fp = Frenet_path()

                lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                fp.t = [t for t in np.arange(0.0, Ti, Ti / 10)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Loongitudinal motion planning (Velocity keeping)
                for tv in np.linspace(TARGET_SPEED_MIN, TARGET_SPEED, N_S_SAMPLE):
                    tfp = copy.deepcopy(fp)
                    lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                    # square of diff from target speed
                    ds = (TARGET_SPEED_ORIGINAL - tfp.s_d[-1]) ** 2

                    tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1] ** 2
                    tfp.cv = KJ * Js + KT * Ti + KD * ds
                    tfp.cf = KLAT * tfp.cd + KLON * tfp.cv
                    tfp.Ti = Ti

                    frenet_paths.append(tfp)

        return frenet_paths


class calc_global_paths_computation:

    def calc_global_paths(self, fplist, csp):

        faTrajX = []
        faTrajY = []

        for fp in fplist:

            # calc global positions
            for i in range(len(fp.s)):
                ix, iy = csp.calc_position(fp.s[i])
                if ix is None:
                    break
                iyaw = csp.calc_yaw(fp.s[i])
                di = fp.d[i]
                fx = ix + di * math.cos(iyaw + math.pi / 2.0)
                fy = iy + di * math.sin(iyaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)

            # Just for plotting
            faTrajX.append(fp.x)
            faTrajY.append(fp.y)

            # calc yaw and ds
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.sqrt(dx**2 + dy**2))

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

            # calc curvature
            for i in range(len(fp.yaw) - 1):
                fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

        return fplist


class check_paths_computation:

    def plot_polygons(
        self, ego_polygons, obstacle_polygons, static_line_obstacles, filename1, filename2, index, collision, time
    ):
        """
        Plots ego polygons, obstacle polygons, and static line obstacles (road boundaries) and saves the plot to two PNG files with reduced quality.
        Adds the index and collision status at the bottom-left corner of the plot.
        Displays the simulation time in the title of the plot.

        Parameters
        ----------
        ego_polygons : list of shapely.geometry.Polygon
            List of ego vehicle polygons at different timesteps.
        obstacle_polygons : list of list of shapely.geometry.Polygon
            Nested list of obstacle polygons at different timesteps.
            For example: [[poly_at_t0, poly_at_t1, ...], [poly2_at_t0, poly2_at_t1, ...], ...]
        static_line_obstacles : list of shapely.geometry.LineString
            List of static line obstacles to plot as road boundaries.
        filename1 : str
            Name of the first output PNG file.
        filename2 : str
            Name of the second output PNG file.
        index : int
            Index to annotate on the plot.
        collision : bool
            Whether there is a collision (True) or not (False).
        time : float
            The simulation time to include in the plot title.
        """
        # Ensure directories exist
        os.makedirs(os.path.dirname(filename1), exist_ok=True)
        os.makedirs(os.path.dirname(filename2), exist_ok=True)

        fig, ax = plt.subplots(figsize=(10, 10))

        # Plot Ego polygons (for each timestep)
        for i, ego_poly in enumerate(ego_polygons):
            if ego_poly.is_empty:
                continue
            x, y = ego_poly.exterior.xy
            ax.fill(x, y, alpha=0.5, fc="blue", ec="blue", label="Ego" if i == 0 else "")

        # Define a color map for obstacles
        cmap = get_cmap("tab10")  # Use a colormap with distinct colors (e.g., 'tab10', 'tab20', etc.)
        num_obstacles = len(obstacle_polygons)
        obstacle_colors = [cmap(i / max(1, num_obstacles - 1)) for i in range(num_obstacles)]

        # Plot obstacle polygons with different colors
        for obstacle_index, obs_list in enumerate(obstacle_polygons):
            color = obstacle_colors[obstacle_index]  # Assign a unique color for this obstacle
            for j, obs_poly in enumerate(obs_list):
                if obs_poly.is_empty:
                    continue
                x, y = obs_poly.exterior.xy
                ax.fill(x, y, alpha=0.3, fc=color, ec=color, label=f"Obstacle {obstacle_index + 1}" if j == 0 else "")

        # Plot static line obstacles (road boundaries) in light grey, alpha=0.5, and thinner line width
        for line_obs in static_line_obstacles:
            x, y = line_obs.xy
            ax.plot(
                x,
                y,
                color="black",
                linestyle="-",
                linewidth=0.5,
                alpha=0.5,
                label="Road Boundary" if static_line_obstacles.index(line_obs) == 0 else "",
            )

        # Annotate the index and collision status on the plot at the bottom-left corner
        collision_status = "COLLISION" if collision else "OK"
        ax.text(
            0.01,
            0.01,
            f"Index: {index} - {collision_status}",
            transform=ax.transAxes,
            fontsize=12,
            color="red" if collision else "green",
            bbox=dict(facecolor="white", alpha=0.5, edgecolor="none"),
        )

        # Update plot title with time
        ax.set_title(f"Ego, Obstacle Polygons, and Road Boundaries (Time: {time:.2f}s)")

        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        # Save two copies of the plot with reduced quality
        plt.savefig(filename1, dpi=50)  # Lower DPI for reduced quality
        plt.savefig(filename2, dpi=50)  # Save the second copy with the same settings
        plt.close(fig)  # Close figure to free up memory

    def check_collision_dynamic(
        self,
        static_line_obstacles: list,  # List of Shapely line obstacles
        fp,
        obstacle_polygons: list[Polygon],  # List of polygon obstacles
        ego_polygons: list[Polygon],  # List of ego vehicle polygons
        marker: bool,
        index_next_collision,
    ) -> bool:
        # Get the polygon for the ego vehicle at this trajectory point
        # Check for collision with any obstacle

        line = LineString([ego_polygons[0].exterior.coords[0], ego_polygons[-1].exterior.coords[0]])

        # last_idx = len(ego_polygons) - 1
        for line_obs in static_line_obstacles:
            if line.intersects(line_obs):
                if marker:
                    # print(f"Collision detected with static line obstacle: {line_obs}")
                    pass
                return True

            ## Added to account for every curve trajectory :
            for i, ego_polygon in enumerate(ego_polygons):
                line_individual_poly = LineString(
                    [ego_polygons[i].exterior.coords[0], ego_polygons[-1].exterior.coords[0]]
                )
                line_individual_poly_2 = LineString(
                    [ego_polygons[i].exterior.coords[2], ego_polygons[-1].exterior.coords[2]]
                )
                if line_individual_poly.intersects(line_obs) or line_individual_poly_2.intersects(line_obs):
                    return True

        ## Check if the last polygon hits the barrier
        for line_obs in static_line_obstacles:
            if ego_polygons[len(ego_polygons) - 1].intersects(line_obs):
                if marker:
                    # print(f"Collision detected with static line obstacle: {line_obs}")
                    pass
                return True

        for i in range(index_next_collision, len(ego_polygons)):
            if ego_polygons[i].intersects(obstacle_polygons[i]):  # Check for intersection
                return True  # Return True immediately if a collision is detected
            if i < len(ego_polygons) - 1:
                line = LineString([ego_polygons[i].exterior.coords[0], ego_polygons[i + 1].exterior.coords[0]])
                line2 = LineString(
                    [obstacle_polygons[i].exterior.coords[0], obstacle_polygons[i + 1].exterior.coords[0]]
                )
                if line.intersects(obstacle_polygons[i]) or line.intersects(line2):  # Check for intersection
                    return True  # Return True immediately if a collision is detected
        return False

    def compute_ego_poly_list(self, sim_obs, fp):
        future_polygons = []
        for i in range(0, len(fp.x)):
            for pname, pdata in sim_obs.players.items():
                if pname == self.name:  # Ignore ego vehicle
                    # Extract current state
                    # print(f"The frenet best path is {fp.s[i]:.3f}, {fp.s[-1]:.3f}, {len(fp.s) == len(fp.x)}")
                    if fp.x[i] is None:
                        raise Exception("stop here!")
                    pred_x = fp.x[i]
                    pred_y = fp.y[i]
                    obs_psi = fp.yaw[i] - fp.yaw[0]
                    current_occupancy = pdata.occupancy  # This is a Shapely Polygon
                    # print(f" \nObstacle {pname}: current occupancy {current_occupancy}")

                    future_polygon = translate(current_occupancy, xoff=pred_x - fp.x[0], yoff=pred_y - fp.y[0])
                    future_polygon = rotate(future_polygon, -pdata.state.psi, origin="centroid", use_radians=True)
                    future_polygon = scale(future_polygon, xfact=1.0, yfact=1.2, origin="centroid")
                    future_polygon = rotate(
                        future_polygon, pdata.state.psi + obs_psi, origin="centroid", use_radians=True
                    )

                    future_polygons.append(future_polygon)
                    # print(f" Obstacle {pname}: new occupancy {future_polygon}\n")
        return future_polygons

    def compute_obstacles_poly(self, sim_obs: SimObservations, obs_t, slope):
        future_polygons = []
        list_pred = []

        number_obstacle = 0
        for pname, pdata in sim_obs.players.items():

            if pname != self.name:  # Ignore ego vehicle
                # Extract current state
                obs_x = pdata.state.x
                obs_y = pdata.state.y
                obs_psi = pdata.state.psi
                obs_vx = pdata.state.vx

                current_occupancy = pdata.occupancy  # This is a Shapely Polygon
                # print(f" \nObstacle {pname}: current occupancy {current_occupancy}")

                number_obstacle += 1
                # if number_obstacle == 2:
                #     print(
                #         "the speed is equal to :",
                #         obs_vx,
                #         "time is equal to :",
                #         sim_obs.time,
                #         "x = :",
                #         obs_x,
                #         "y = :",
                #         obs_y,
                #     )
                # Predict future position
                if self.sense > 0:  # Cars are going right
                    pred_x = obs_x + np.cos(np.arctan(slope)) * obs_vx * obs_t
                    pred_y = obs_y + np.sin(np.arctan(slope)) * obs_vx * obs_t
                else:  # Cars are going left
                    pred_x = obs_x - np.cos(np.arctan(slope)) * obs_vx * obs_t
                    pred_y = obs_y - np.sin(np.arctan(slope)) * obs_vx * obs_t

                # Translate and rotate the polygon to the predicted position
                future_polygon = translate(current_occupancy, xoff=pred_x - obs_x, yoff=pred_y - obs_y)
                future_polygon = rotate(future_polygon, 0, origin=(pred_x, pred_y), use_radians=True)
                # future_polygon = scale(future_polygon, xfact=1.1, yfact=1.1, origin="centroid")

                future_polygons.append(future_polygon)

                list_pred.append((pred_x, pred_y))

        # print(f" Obstacle {pname}: new occupancy {future_polygon}\n")
        return future_polygons, list_pred

    def check_paths(
        self, static_line_obstacles, fplist, ob, sim_obs, name, sense, slope, index_next_collision, time_since_simul
    ):
        self.name = name
        self.sense = sense

        okind = []
        for i, _ in enumerate(fplist):
            # if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            # continue
            # elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            # continue
            # elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            # continue
            size_path = len(fplist[i].s)
            # Pdm4arAgent_instance = Pdm4arAgent()
            obstacle_polygones_predict = []
            prediction_test_tot = []
            obstacle_number = 0
            for _ in ob:
                prediction_test = []
                list_obstacle = []
                for index in range(size_path):
                    time = ((fplist[i].Ti - time_since_simul) / size_path) * index
                    future_polygone, pred = self.compute_obstacles_poly(sim_obs, time, slope)
                    prediction_test.append(pred[obstacle_number])
                    polygone_for_list = future_polygone[obstacle_number]
                    list_obstacle.append(polygone_for_list)
                obstacle_polygones_predict.append(list_obstacle)
                obstacle_number += 1
                prediction_test_tot.append(prediction_test)
            # print("Obstacles: ", obstacle_polygones_predict, "\n")

            obstacle_number = 0
            final_answer = False
            polygons_ego = self.compute_ego_poly_list(sim_obs, fplist[i])
            # print("Ego: ", polygons_ego, "\n")
            marker = False
            if i == 146:
                marker = True

            for _ in ob:

                answer = self.check_collision_dynamic(
                    static_line_obstacles,
                    fplist[i],
                    obstacle_polygones_predict[obstacle_number],
                    polygons_ego,
                    marker,
                    index_next_collision,
                )
                if answer is True and final_answer is False:
                    final_answer = True
                # if i == 247:
                #     print(
                #         "\n\n\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX PAY ATTENTION XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
                #     )
                #     print("fplist[i]", fplist[i])
                #     print("obstacle_polygones_predict[obstacle_number]", obstacle_polygones_predict[obstacle_number])
                #     print("polygons_ego: ", polygons_ego)
                #     print("Final answer: ", final_answer)
                #     print(
                #         "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX PAY ATTENTION XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n\n"
                #     )
                obstacle_number += 1

            # Plot and save polygons
            filename1 = f"src/pdm4ar/exercises/ex12/A_live_plot.png"
            filename2 = f"src/pdm4ar/exercises/ex12/A_log/plot_{i}.png"
            time = sim_obs.time

            """
            if sim_obs.time >= 3:
                self.plot_polygons(
                    polygons_ego,
                    obstacle_polygones_predict,
                    static_line_obstacles,
                    filename1,
                    filename2,
                    i,
                    final_answer,
                    time,
                )
            """

            if final_answer is True:
                # print("there is collision", i, "/", len(fplist))
                continue

            # ---------- ADDED CODE STARTS HERE ----------
            # Compute the minimum distance to obstacles and penalize if below threshold
            min_distance = float("inf")
            # For every step in the path
            for step_idx, ego_poly in enumerate(polygons_ego):
                # Check distance to each obstacle's polygon at the same step
                for obstacle_idx in range(len(obstacle_polygones_predict)):
                    dist = ego_poly.distance(obstacle_polygones_predict[obstacle_idx][step_idx])
                    if dist < min_distance:
                        min_distance = dist

            # If min distance is below the threshold, add a large penalty to the path cost
            fplist[i].cf += (1 / min_distance) * 1000

            # ---------- ADDED CODE ENDS HERE ----------

            okind.append(i)

        return [fplist[i] for i in okind], prediction_test_tot


class frenet_optimal_planning_computation:
    def frenet_optimal_planning(
        self,
        static_line_obstacles,
        csp,
        s0,
        c_speed,
        c_d,
        c_d_d,
        c_d_dd,
        ob,
        sim_obs,
        name,
        sense,
        slope,
        index_next_collision,
        time_since_simul,
    ):
        global MINT, MAXT, DT, D_T_S, D_ROAD_W, N_S_SAMPLE, TARGET_SPEED, TARGET_SPEED_MIN, MAX_ROAD_WIDTH
        if MINT <= 0.5 and D_ROAD_W == 27:
            MINT = 4.0
            MAXT = 6.0
            DT = 0.5
            D_ROAD_W = ORIGINAL_D_ROAD_W
            N_S_SAMPLE = ORIGINAL_N_S_SAMPLE
            TARGET_SPEED = 25
            TARGET_SPEED_MIN = 0
            return None, None, None

        fpplist = []
        # pdb.set_trace()

        ## Adjusted for the code :
        frenet_path_computer = calc_frenet_paths_computation()
        global_path_computer = calc_global_paths_computation()
        path_checker = check_paths_computation()

        fplist = frenet_path_computer.calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
        fplist = global_path_computer.calc_global_paths(fplist, csp)
        path_testing = fplist
        fplist, prediction_test = path_checker.check_paths(
            static_line_obstacles, fplist, ob, sim_obs, name, sense, slope, index_next_collision, time_since_simul
        )

        # fpplist = deepcopy(fplist)
        fpplist.extend(fplist)
        if len(fpplist) != 0:
            MINT = 4.0
            MAXT = 6.0
            DT = 0.5
            D_ROAD_W = ORIGINAL_D_ROAD_W
            N_S_SAMPLE = ORIGINAL_N_S_SAMPLE
            TARGET_SPEED = 25
            TARGET_SPEED_MIN = 0
            # find minimum cost path
            mincost = float("inf")
            bestpath = None
            for fp in fplist:
                if mincost >= fp.cf:
                    mincost = fp.cf
                    bestpath = fp

            index = 0
            for test in path_testing:
                if test.cf == bestpath.cf:
                    print("\nThe index of the path is :", index)
                index += 1
            return bestpath, path_testing, prediction_test

        else:
            if MINT <= 0.5:
                D_ROAD_W = 3 * D_ROAD_W
                N_S_SAMPLE = 3 * N_S_SAMPLE
                TARGET_SPEED = TARGET_SPEED
                TARGET_SPEED_MIN = TARGET_SPEED_MIN
                print("testing new values for MINT and MAXT", MINT, MAXT, D_ROAD_W, N_S_SAMPLE, MAX_ROAD_WIDTH)

            else:
                MAXT = MINT
                MINT = MINT - 0.5
                TARGET_SPEED = TARGET_SPEED + 5
                TARGET_SPEED_MIN = TARGET_SPEED_MIN - 7
                print("testing new values for MINT and MAXT", MINT, MAXT, D_ROAD_W, N_S_SAMPLE, MAX_ROAD_WIDTH)

            bestpath, path_testing, prediction_test = self.frenet_optimal_planning(
                static_line_obstacles,
                csp,
                s0,
                c_speed,
                c_d,
                c_d_d,
                c_d_dd,
                ob,
                sim_obs,
                name,
                sense,
                slope,
                index_next_collision,
                time_since_simul,
            )
            return bestpath, path_testing, prediction_test


class generate_target_course_computation:
    def generate_target_course(self, x, y):
        csp = CubicSpline2D(x, y)
        s = np.arange(0, csp.s[-1], 0.1)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = csp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(csp.calc_yaw(i_s))
            rk.append(csp.calc_curvature(i_s))

        return rx, ry, ryaw, rk, csp
