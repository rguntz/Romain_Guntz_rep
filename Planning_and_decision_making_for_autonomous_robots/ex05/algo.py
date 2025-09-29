from collections.abc import Sequence
from turtle import forward, right

from dg_commons import SE2Transform

from pdm4ar.exercises.ex05.structures import *
from pdm4ar.exercises_def.ex05.utils import extract_path_points
import math


class PathPlanner(ABC):
    @abstractmethod
    def compute_path(self, start: SE2Transform, end: SE2Transform) -> Sequence[SE2Transform]:
        pass


class Dubins(PathPlanner):
    def __init__(self, params: DubinsParam):
        self.params = params

    def compute_path(self, start: SE2Transform, end: SE2Transform) -> list[SE2Transform]:
        """Generates an optimal Dubins path between start and end configuration

        :param start: the start configuration of the car (x,y,theta)
        :param end: the end configuration of the car (x,y,theta)

        :return: a list[SE2Transform] of configurations in the optimal path the car needs to follow
        """

        path = calculate_dubins_path(
            start_config=start, end_config=end, radius=self.params.min_radius
        )  ## list of segment
        se2_list = extract_path_points(path)
        return se2_list


class ReedsShepp(PathPlanner):
    def __init__(self, params: DubinsParam):
        self.params = params

    def compute_path(self, start: SE2Transform, end: SE2Transform) -> Sequence[SE2Transform]:
        """Generates a Reeds-Shepp *inspired* optimal path between start and end configuration

        :param start: the start configuration of the car (x,y,theta)
        :param end: the end configuration of the car (x,y,theta)

        :return: a list[SE2Transform] of configurations in the optimal path the car needs to follow
        """
        path = calculate_reeds_shepp_path(start_config=start, end_config=end, radius=self.params.min_radius)
        se2_list = extract_path_points(path)
        return se2_list


def calculate_car_turning_radius(wheel_base: float, max_steering_angle: float) -> DubinsParam:
    # TODO implement here your solution

    min_radius = wheel_base / np.tan(max_steering_angle)

    return DubinsParam(min_radius=min_radius)


def calculate_turning_circles(current_config: SE2Transform, radius: float) -> TurningCircle:
    theta = current_config.theta
    [x, y] = current_config.p

    left_circle_center = SE2Transform([x - radius * np.sin(theta), y + radius * np.cos(theta)], 0)
    right_circle_center = SE2Transform([x + radius * np.sin(theta), y - radius * np.cos(theta)], 0)

    right_circle = Curve.create_circle(
        center=right_circle_center,
        config_on_circle=right_circle_center,
        radius=radius,
        curve_type=DubinsSegmentType.RIGHT,
    )

    left_circle = Curve.create_circle(
        center=left_circle_center,
        config_on_circle=left_circle_center,
        radius=radius,
        curve_type=DubinsSegmentType.LEFT,
    )

    return TurningCircle(left=left_circle, right=right_circle)


def dist(C1, C2):
    return np.linalg.norm(C1 - C2)


def calculate_tangent_btw_circles(circle_start: Curve, circle_end: Curve) -> list[Line]:
    # TODO implement here your solution
    tangent_lines = []

    direction_of_circle_start = circle_start.type
    direction_of_circle_end = circle_end.type

    C1 = np.array(circle_start.center.p)
    C2 = np.array(circle_end.center.p)

    r1 = circle_start.radius
    r2 = circle_end.radius

    if direction_of_circle_start == direction_of_circle_end:
        r3 = r1 - r2

        # Calculate distances
        hypotenuse = dist(C1, C2)
        short = r3

        if np.array_equal(C1, C2):
            return []

        # Calculate angles
        phi = np.arccos(short / hypotenuse)
        phi1 = np.arctan2(C2[1] - C1[1], C2[0] - C1[0]) + phi

        # Tangent points for the first circle
        t1x = C1[0] + r1 * np.cos(phi1)
        t1y = C1[1] + r1 * np.sin(phi1)

        # Tangent points for the second circle
        t2x = C2[0] + r2 * np.cos(phi1)
        t2y = C2[1] + r2 * np.sin(phi1)

        # Second angle for the other tangent points
        phi2 = np.arctan2(C2[1] - C1[1], C2[0] - C1[0]) - phi

        # Second set of tangent points for the first circle
        s1x = C1[0] + r1 * np.cos(phi2)
        s1y = C1[1] + r1 * np.sin(phi2)

        # Second set of tangent points for the second circle
        s2x = C2[0] + r2 * np.cos(phi2)
        s2y = C2[1] + r2 * np.sin(phi2)

        if direction_of_circle_start == DubinsSegmentType.RIGHT:

            tangent_lines.append(
                Line(
                    start_config=SE2Transform([t1x, t1y], math.atan2(t2y - t1y, t2x - t1x)),
                    end_config=SE2Transform([t2x, t2y], math.atan2(t2y - t1y, t2x - t1x)),
                    gear=Gear.FORWARD,
                )
            )

        elif direction_of_circle_start == DubinsSegmentType.LEFT:

            tangent_lines.append(
                Line(
                    start_config=SE2Transform([s1x, s1y], math.atan2(s2y - s1y, s2x - s1x)),
                    end_config=SE2Transform([s2x, s2y], math.atan2(s2y - s1y, s2x - s1x)),
                    gear=Gear.FORWARD,
                )
            )

    else:

        # Calculate distances
        hypotenuse = dist(C1, C2)
        short = r1 + r2

        if hypotenuse < (r1 + r2):
            return []

        # Calculate angles
        phi = np.arctan2(C2[1] - C1[1], C2[0] - C1[0]) + np.arcsin(short / hypotenuse) - np.pi / 2

        # Tangent points for the first circle
        t1x = C1[0] + r1 * np.cos(phi)
        t1y = C1[1] + r1 * np.sin(phi)

        # Tangent points for the second circle
        t2x = C2[0] + r2 * np.cos(phi + np.pi)
        t2y = C2[1] + r2 * np.sin(phi + np.pi)

        # Second angle for the other tangent points
        phi2 = np.arctan2(C2[1] - C1[1], C2[0] - C1[0]) - np.arcsin(short / hypotenuse) + np.pi / 2

        # Second set of tangent points for the first circle
        s1x = C1[0] + r1 * np.cos(phi2)
        s1y = C1[1] + r1 * np.sin(phi2)

        # Second set of tangent points for the second circle
        s2x = C2[0] + r2 * np.cos(phi2 + np.pi)
        s2y = C2[1] + r2 * np.sin(phi2 + np.pi)

        if direction_of_circle_start == DubinsSegmentType.LEFT:
            tangent_lines.append(
                Line(
                    start_config=SE2Transform([t1x, t1y], math.atan2(t2y - t1y, t2x - t1x)),
                    end_config=SE2Transform([t2x, t2y], math.atan2(t2y - t1y, t2x - t1x)),
                    gear=Gear.FORWARD,
                )
            )

        else:
            tangent_lines.append(
                Line(
                    start_config=SE2Transform([s1x, s1y], math.atan2(s2y - s1y, s2x - s1x)),
                    end_config=SE2Transform([s2x, s2y], math.atan2(s2y - s1y, s2x - s1x)),
                    gear=Gear.FORWARD,
                )
            )

    return tangent_lines  # i.e., [Line(),...]


def angle_between_points(radius, initial_point, final_point, direction, center):

    center = np.array(center.p)

    initial_point = initial_point - center
    final_point = final_point - center

    # Calculate angles of the points
    theta_initial = np.arctan2(initial_point[1], initial_point[0])
    theta_final = np.arctan2(final_point[1], final_point[0])

    # Calculate the angle difference
    angle_difference = theta_final - theta_initial

    # Adjust the angle difference based on the direction
    if direction == -1:  # Clockwise => right
        if angle_difference > 0:
            angle_difference -= 2 * np.pi
    elif direction == 1:  # Anticlockwise => left
        if angle_difference < 0:
            angle_difference += 2 * np.pi

    return abs(angle_difference)


def path_creator(
    start_config, end_config, radius, cicle_start, circle_end, direction_first_circle, direction_second_circle
):
    path_segments = []

    tangent = calculate_tangent_btw_circles(cicle_start, circle_end)
    if len(tangent) != 0:
        for tangent in tangent:
            angle = angle_between_points(
                radius, start_config.p, np.array(tangent.start_config.p), direction_first_circle, cicle_start.center
            )
            curve_segment = Curve(
                start_config=start_config,
                end_config=tangent.start_config,
                center=cicle_start.center,
                radius=radius,
                curve_type=direction_first_circle,
                arc_angle=angle,  # This can be adjusted based on the actual curve segment
            )

            path_segments.append(curve_segment)
            path_segments.append(tangent)

            angle = angle_between_points(
                radius, np.array(tangent.end_config.p), end_config.p, direction_second_circle, circle_end.center
            )
            curve_segment = Curve(
                start_config=tangent.end_config,
                end_config=end_config,
                center=circle_end.center,
                radius=radius,
                curve_type=direction_second_circle,
                arc_angle=angle,  # This can be adjusted based on the actual curve segment
            )

            path_segments.append(curve_segment)

    return path_segments


def segment_path_length_calculator(segment_path, original_path, original_length_path):
    length_path = 0
    for element in segment_path:
        length_path = length_path + abs(element.length)
    if length_path < original_length_path and len(segment_path) != 0:
        original_length_path = length_path
        original_path = segment_path

    return original_path, original_length_path


def calculate_dubins_path(start_config: SE2Transform, end_config: SE2Transform, radius: float) -> Path:

    # TODO implement here your solution
    # Please keep segments with zero length in the return list & return a valid dubins path!

    original_length_path = np.inf
    original_path = []

    circles_start = calculate_turning_circles(start_config, radius)
    circles_end = calculate_turning_circles(end_config, radius)

    V = np.empty((2, 2), dtype=object)

    circles_start_left = circles_start.left
    circles_start_right = circles_start.right

    circles_end_left = circles_end.left
    circles_end_right = circles_end.right

    V[0][0] = circles_start_left
    V[1][0] = circles_start_right
    V[0][1] = circles_end_left
    V[1][1] = circles_end_right

    for i in range(2):
        for j in range(2):

            ## LSR
            cicle_start = V[i][0]
            circle_end = V[j][1]

            if i == 0:
                direction_first_circle = DubinsSegmentType.LEFT
            elif i == 1:
                direction_first_circle = DubinsSegmentType.RIGHT
            if j == 0:
                direction_second_circle = DubinsSegmentType.LEFT
            elif j == 1:
                direction_second_circle = DubinsSegmentType.RIGHT

            segment_path = path_creator(
                start_config,
                end_config,
                radius,
                cicle_start,
                circle_end,
                direction_first_circle,
                direction_second_circle,
            )
            original_path, original_length_path = segment_path_length_calculator(
                segment_path, original_path, original_length_path
            )

    ## To do at the end

    ## Case of LRL

    cicle_start = circles_start_left
    circle_end = circles_end_left
    ## Initialize the centers
    center1 = circles_start_left.center.p
    center2 = circles_end_left.center.p
    center1 = (center1[0], center1[1])
    center2 = (center2[0], center2[1])

    results = create_new_circle_center(center1, center2, radius)
    if results:
        third_circle, intersection1, intersection2 = results
        intermediate_circle = Curve.create_circle(
            center=SE2Transform([third_circle[0], third_circle[1]], theta=0),
            config_on_circle=SE2Transform([third_circle[0], third_circle[1]], theta=0),
            radius=radius,
            curve_type=DubinsSegmentType.RIGHT,
        )

        for reverse_first_circle in [-1, 1]:
            for reverse_second_circle in [-1, 1]:
                for reverse_intermediate_circle in [-1, 1]:
                    gear1 = 1
                    gear2 = -1
                    angle1_rad, angle2_rad = theta_intersections(
                        third_circle, intersection1, intersection2, gear1, gear2
                    )

                    intersection_1_object = SE2Transform([intersection1[0], intersection1[1]], angle1_rad)
                    intersection_2_object = SE2Transform([intersection2[0], intersection2[1]], angle2_rad)

                    segment_path = path_creator_sheep_CCC_motion(
                        start_config,
                        end_config,
                        radius,
                        cicle_start,
                        circle_end,
                        intermediate_circle,
                        reverse_first_circle,
                        reverse_second_circle,
                        reverse_intermediate_circle,
                        intersection_1_object,
                        intersection_2_object,
                    )

                    obj_list = segment_path
                    if all(obj.gear == Gear.FORWARD for obj in obj_list):
                        original_path, original_length_path = segment_path_length_calculator(
                            segment_path, original_path, original_length_path
                        )

    ## Case of RLR

    cicle_start = circles_start_right
    circle_end = circles_end_right
    ## Initialize the centers
    center1 = circles_start_right.center.p
    center2 = circles_end_right.center.p
    center1 = (center1[0], center1[1])
    center2 = (center2[0], center2[1])

    results = create_new_circle_center(center1, center2, radius)
    if results:
        third_circle, intersection1, intersection2 = results

        intermediate_circle = Curve.create_circle(
            center=SE2Transform([third_circle[0], third_circle[1]], theta=0),
            config_on_circle=SE2Transform([third_circle[0], third_circle[1]], theta=0),
            radius=radius,
            curve_type=DubinsSegmentType.LEFT,
        )

        for reverse_first_circle in [-1, 1]:
            for reverse_second_circle in [-1, 1]:
                for reverse_intermediate_circle in [-1, 1]:
                    gear1 = -1
                    gear2 = 1
                    angle1_rad, angle2_rad = theta_intersections(
                        third_circle, intersection1, intersection2, gear1, gear2
                    )

                    intersection_1_object = SE2Transform([intersection1[0], intersection1[1]], angle1_rad)
                    intersection_2_object = SE2Transform([intersection2[0], intersection2[1]], angle2_rad)

                    segment_path = path_creator_sheep_CCC_motion(
                        start_config,
                        end_config,
                        radius,
                        cicle_start,
                        circle_end,
                        intermediate_circle,
                        reverse_first_circle,
                        reverse_second_circle,
                        reverse_intermediate_circle,
                        intersection_1_object,
                        intersection_2_object,
                    )

                    obj_list = segment_path
                    if all(obj.gear == Gear.FORWARD for obj in obj_list):
                        original_path, original_length_path = segment_path_length_calculator(
                            segment_path, original_path, original_length_path
                        )

    return original_path  # e.g., [Curve(), Line(),..]


def calculate_tangent_btw_circles_sheep(circle_start: Curve, circle_end: Curve) -> list[Line]:
    # TODO implement here your solution
    tangent_lines = []

    direction_of_circle_start = circle_start.type
    direction_of_circle_end = circle_end.type

    C1 = np.array(circle_start.center.p)
    C2 = np.array(circle_end.center.p)

    r1 = circle_start.radius
    r2 = circle_end.radius

    if direction_of_circle_start == direction_of_circle_end:
        r3 = r1 - r2

        # Calculate distances
        hypotenuse = dist(C1, C2)
        short = r3

        if np.array_equal(C1, C2):
            return []

        # Calculate angles
        phi = np.arccos(short / hypotenuse)
        phi1 = np.arctan2(C2[1] - C1[1], C2[0] - C1[0]) + phi

        # Tangent points for the first circle
        t1x = C1[0] + r1 * np.cos(phi1)
        t1y = C1[1] + r1 * np.sin(phi1)

        # Tangent points for the second circle
        t2x = C2[0] + r2 * np.cos(phi1)
        t2y = C2[1] + r2 * np.sin(phi1)

        # Second angle for the other tangent points
        phi2 = np.arctan2(C2[1] - C1[1], C2[0] - C1[0]) - phi

        # Second set of tangent points for the first circle
        s1x = C1[0] + r1 * np.cos(phi2)
        s1y = C1[1] + r1 * np.sin(phi2)

        # Second set of tangent points for the second circle
        s2x = C2[0] + r2 * np.cos(phi2)
        s2y = C2[1] + r2 * np.sin(phi2)

        if direction_of_circle_start == DubinsSegmentType.RIGHT:  ## Right Right

            tangent_lines.append(
                Line(
                    start_config=SE2Transform([t1x, t1y], math.atan2(t2y - t1y, t2x - t1x)),
                    end_config=SE2Transform([t2x, t2y], math.atan2(t2y - t1y, t2x - t1x)),
                    gear=Gear.FORWARD,
                )
            )

            tangent_lines.append(
                Line(
                    start_config=SE2Transform([s1x, s1y], math.atan2(-s2y + s1y, -s2x + s1x)),
                    end_config=SE2Transform([s2x, s2y], math.atan2(-s2y + s1y, -s2x + s1x)),
                    gear=Gear.REVERSE,
                )
            )

        elif direction_of_circle_start == DubinsSegmentType.LEFT:  ## Left
            tangent_lines.append(
                Line(
                    start_config=SE2Transform([s1x, s1y], math.atan2(s2y - s1y, s2x - s1x)),
                    end_config=SE2Transform([s2x, s2y], math.atan2(s2y - s1y, s2x - s1x)),
                    gear=Gear.FORWARD,
                )
            )

            tangent_lines.append(
                Line(
                    start_config=SE2Transform([t1x, t1y], math.atan2(-t2y + t1y, -t2x + t1x)),
                    end_config=SE2Transform([t2x, t2y], math.atan2(-t2y + t1y, -t2x + t1x)),
                    gear=Gear.REVERSE,
                )
            )

    else:

        # Calculate distances
        hypotenuse = dist(C1, C2)
        short = r1 + r2

        if hypotenuse < (r1 + r2):
            return []

        # Calculate angles
        phi = np.arctan2(C2[1] - C1[1], C2[0] - C1[0]) + np.arcsin(short / hypotenuse) - np.pi / 2

        # Tangent points for the first circle
        t1x = C1[0] + r1 * (np.cos(phi))
        t1y = C1[1] + r1 * (np.sin(phi))

        # Tangent points for the second circle
        t2x = C2[0] + r2 * (np.cos(phi + np.pi))
        t2y = C2[1] + r2 * (np.sin(phi + np.pi))

        # Second angle for the other tangent points
        phi2 = np.arctan2(C2[1] - C1[1], C2[0] - C1[0]) - np.arcsin(short / hypotenuse) + np.pi / 2

        # Second set of tangent points for the first circle
        s1x = C1[0] + r1 * (np.cos(phi2))
        s1y = C1[1] + r1 * (np.sin(phi2))

        # Second set of tangent points for the second circle
        s2x = C2[0] + r2 * (np.cos(phi2 + np.pi))
        s2y = C2[1] + r2 * (np.sin(phi2 + np.pi))

        if direction_of_circle_start == DubinsSegmentType.LEFT:  ## LEFT TO RIGHT
            tangent_lines.append(
                Line(
                    start_config=SE2Transform([t1x, t1y], math.atan2(t2y - t1y, t2x - t1x)),
                    end_config=SE2Transform([t2x, t2y], math.atan2(t2y - t1y, t2x - t1x)),
                    gear=Gear.FORWARD,
                )
            )

            tangent_lines.append(
                Line(
                    start_config=SE2Transform([s1x, s1y], math.atan2(-s2y + s1y, -s2x + s1x)),
                    end_config=SE2Transform([s2x, s2y], math.atan2(-s2y + s1y, -s2x + s1x)),
                    gear=Gear.REVERSE,
                )
            )

        else:  ## RIGHT TO LEFT
            tangent_lines.append(
                Line(
                    start_config=SE2Transform([s1x, s1y], math.atan2(s2y - s1y, s2x - s1x)),
                    end_config=SE2Transform([s2x, s2y], math.atan2(s2y - s1y, s2x - s1x)),
                    gear=Gear.FORWARD,
                )
            )

            tangent_lines.append(
                Line(
                    start_config=SE2Transform([t1x, t1y], math.atan2(-t2y + t1y, -t2x + t1x)),
                    end_config=SE2Transform([t2x, t2y], math.atan2(-t2y + t1y, -t2x + t1x)),
                    gear=Gear.REVERSE,
                )
            )

    return tangent_lines  # i.e., [Line(),...]


def path_creator_sheep(
    start_config,
    end_config,
    radius,
    cicle_start,
    circle_end,
    direction_first_circle,
    direction_second_circle,
    gear_tangent,
):

    # reverse_sense_first_circle
    if direction_first_circle == cicle_start.type:
        gear1 = Gear.FORWARD
    else:
        gear1 = Gear.REVERSE
    # reverse_sense_second_circle
    if direction_second_circle == circle_end.type:
        gear2 = Gear.FORWARD
    else:
        gear2 = Gear.REVERSE

    path_segments = []

    tangent = calculate_tangent_btw_circles_sheep(cicle_start, circle_end)
    if len(tangent) != 0:
        for tangent_element in tangent:
            if tangent_element.gear == gear_tangent:
                tangent = tangent_element

        angle = angle_between_points(
            radius, start_config.p, np.array(tangent.start_config.p), direction_first_circle, cicle_start.center
        )
        curve_segment = Curve(
            start_config=start_config,
            end_config=tangent.start_config,
            center=cicle_start.center,
            radius=radius,
            curve_type=cicle_start.type,
            arc_angle=angle,  # This can be adjusted based on the actual curve segment
            gear=gear1,
        )

        path_segments.append(curve_segment)
        path_segments.append(tangent)

        angle = angle_between_points(
            radius, np.array(tangent.end_config.p), end_config.p, direction_second_circle, circle_end.center
        )
        curve_segment = Curve(
            start_config=tangent.end_config,
            end_config=end_config,
            center=circle_end.center,
            radius=radius,
            curve_type=circle_end.type,
            arc_angle=angle,  # This can be adjusted based on the actual curve segment
            gear=gear2,
        )

        path_segments.append(curve_segment)

    return path_segments


def anticlockwise_angle(x, y):
    # Get the counterclockwise angle from the positive x-axis
    angle = math.atan2(y, x)

    # Ensure the angle is within [0, 2π] range
    anticlockwise_angle = angle % (2 * math.pi)

    # Return the angle in radians
    return anticlockwise_angle


def create_new_circle_center(center1, center2, radius):
    center_1_centered = (center1[0] - center1[0], center1[1] - center1[1])
    center_2_centered = (center2[0] - center1[0], center2[1] - center1[1])

    if np.linalg.norm(center_2_centered) <= 4 * radius:

        theta = anticlockwise_angle(center_2_centered[0], center_2_centered[1])
        R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
        D = np.linalg.norm(center_2_centered)
        x = np.sqrt(4 * (radius**2) - (1 / 4) * (D**2)) if (4 * (radius**2) - (1 / 4) * (D**2)) >= 0 else 0
        center_third = np.array([D / 2, x, 0])  # Convert to a 3D vector

        center_third = np.dot(R, center_third)  # or use `R @ center_third`
        center_third = (center_third[0], center_third[1])
        center_third = (center_third[0] + center1[0], center_third[1] + center1[1])

        intersection1 = ((center1[0] + center_third[0]) / 2, (center1[1] + center_third[1]) / 2)
        intersection2 = ((center2[0] + center_third[0]) / 2, (center2[1] + center_third[1]) / 2)

    else:
        return []

    return center_third, intersection1, intersection2


def path_creator_sheep_CCC_motion(
    start_config,
    end_config,
    radius,
    cicle_start,
    circle_end,
    circle_intermediate,
    direction_first_circle,
    direction_second_circle,
    direction_intermediate_circle,
    intersection_1_object,
    intersection_2_object,
):

    path_segments = []

    # reverse_sense_first_circle
    if direction_first_circle == cicle_start.type:
        gear1 = Gear.FORWARD
    else:
        gear1 = Gear.REVERSE
    # reverse_sense_second_circle
    if direction_second_circle == circle_end.type:
        gear2 = Gear.FORWARD
    else:
        gear2 = Gear.REVERSE
    if direction_intermediate_circle == circle_intermediate.type:
        gear_intermediate = Gear.FORWARD
    else:
        gear_intermediate = Gear.REVERSE

    ## Circle 1

    angle = angle_between_points(
        radius, np.array(start_config.p), np.array(intersection_1_object.p), direction_first_circle, cicle_start.center
    )
    curve_segment = Curve(
        start_config=start_config,
        end_config=intersection_1_object,
        center=cicle_start.center,
        radius=radius,
        curve_type=cicle_start.type,
        arc_angle=angle,
        gear=gear1,
    )
    path_segments.append(curve_segment)

    ## Circle 2

    angle = angle_between_points(
        radius,
        np.array(intersection_1_object.p),
        np.array(intersection_2_object.p),
        direction_intermediate_circle,
        circle_intermediate.center,
    )

    curve_segment = Curve(
        start_config=intersection_1_object,
        end_config=intersection_2_object,
        center=circle_intermediate.center,
        radius=radius,
        curve_type=circle_intermediate.type,
        arc_angle=angle,
        gear=gear_intermediate,
    )
    path_segments.append(curve_segment)

    ## Circle 3
    angle = angle_between_points(
        radius, np.array(intersection_2_object.p), np.array(end_config.p), direction_second_circle, circle_end.center
    )
    curve_segment = Curve(
        start_config=intersection_2_object,
        end_config=end_config,
        center=circle_end.center,
        radius=radius,
        curve_type=circle_end.type,
        arc_angle=angle,
        gear=gear2,
    )
    path_segments.append(curve_segment)

    return path_segments


def theta_intersections(center_third, intersection1, intersection2, gear1, gear2):

    vecteur1 = (center_third[0] - intersection1[0], center_third[1] - intersection1[1])
    vecteur2 = (intersection2[0] - center_third[0], intersection2[1] - center_third[1])

    # Compute the angles in radians
    angle1_rad = np.arctan2(vecteur1[1], vecteur1[0])  # Angle for vecteur1
    angle2_rad = np.arctan2(vecteur2[1], vecteur2[0])  # Angle for vecteur2

    # Normalize the angles to be in the range [0, 2π)
    angle1_rad = angle1_rad % (2 * np.pi)
    angle2_rad = angle2_rad % (2 * np.pi)

    # Rotate angles based on gear values
    if gear1 == 1:
        angle1_rad += np.pi / 2  # Rotate anticlockwise by π/2
    elif gear1 == -1:
        angle1_rad -= np.pi / 2  # Rotate clockwise by π/2

    if gear2 == 1:
        angle2_rad += np.pi / 2  # Rotate anticlockwise by π/2
    elif gear2 == -1:
        angle2_rad -= np.pi / 2  # Rotate clockwise by π/2

    # Normalize the angles again to ensure they are within [0, 2π)
    angle1_rad = angle1_rad % (2 * np.pi)
    angle2_rad = angle2_rad % (2 * np.pi)

    return angle1_rad, angle2_rad


def calculate_reeds_shepp_path(start_config: SE2Transform, end_config: SE2Transform, radius: float) -> Path:

    original_length_path = np.inf
    original_path = []

    circles_start = calculate_turning_circles(start_config, radius)
    circles_end = calculate_turning_circles(end_config, radius)

    V = np.empty((2, 2), dtype=object)

    circles_start_left = circles_start.left
    circles_start_right = circles_start.right

    circles_end_left = circles_end.left
    circles_end_right = circles_end.right

    V[0][0] = circles_start_left
    V[1][0] = circles_start_right
    V[0][1] = circles_end_left
    V[1][1] = circles_end_right

    for i in range(2):
        for j in range(2):
            ## Account for left or right circles
            cicle_start = V[i][0]
            circle_end = V[j][1]

            for reverse_gear_tangent in [-1, 1]:  ## Account for reverse of normal tangent
                ## 2 cases for each gear of tangent : one where we follow direction of circle, one in the opposite

                for revese_gear_first_circle in [-1, 1]:
                    for revese_gear_second_circle in [-1, 1]:

                        segment_path = path_creator_sheep(
                            start_config,
                            end_config,
                            radius,
                            cicle_start,
                            circle_end,
                            revese_gear_first_circle,
                            revese_gear_second_circle,
                            reverse_gear_tangent,
                        )
                        obj_list = segment_path
                        if all(obj.gear == obj_list[0].gear for obj in obj_list):
                            original_path, original_length_path = segment_path_length_calculator(
                                segment_path, original_path, original_length_path
                            )

    ## Case of LRL

    cicle_start = circles_start_left
    circle_end = circles_end_left
    ## Initialize the centers
    center1 = circles_start_left.center.p
    center2 = circles_end_left.center.p
    center1 = (center1[0], center1[1])
    center2 = (center2[0], center2[1])

    results = create_new_circle_center(center1, center2, radius)
    if results:
        third_circle, intersection1, intersection2 = results

        intermediate_circle = Curve.create_circle(
            center=SE2Transform([third_circle[0], third_circle[1]], theta=0),
            config_on_circle=SE2Transform([third_circle[0], third_circle[1]], theta=0),
            radius=radius,
            curve_type=DubinsSegmentType.RIGHT,
        )

        for reverse_first_circle in [-1, 1]:
            for reverse_second_circle in [-1, 1]:
                for reverse_intermediate_circle in [-1, 1]:
                    gear1 = 1
                    gear2 = -1
                    angle1_rad, angle2_rad = theta_intersections(
                        third_circle, intersection1, intersection2, gear1, gear2
                    )

                    intersection_1_object = SE2Transform([intersection1[0], intersection1[1]], angle1_rad)
                    intersection_2_object = SE2Transform([intersection2[0], intersection2[1]], angle2_rad)

                    segment_path = path_creator_sheep_CCC_motion(
                        start_config,
                        end_config,
                        radius,
                        cicle_start,
                        circle_end,
                        intermediate_circle,
                        reverse_first_circle,
                        reverse_second_circle,
                        reverse_intermediate_circle,
                        intersection_1_object,
                        intersection_2_object,
                    )

                    obj_list = segment_path
                    if all(obj.gear == obj_list[0].gear for obj in obj_list):
                        original_path, original_length_path = segment_path_length_calculator(
                            segment_path, original_path, original_length_path
                        )

    ## Case of RLR

    cicle_start = circles_start_right
    circle_end = circles_end_right
    ## Initialize the centers
    center1 = circles_start_right.center.p
    center2 = circles_end_right.center.p
    center1 = (center1[0], center1[1])
    center2 = (center2[0], center2[1])

    results = create_new_circle_center(center1, center2, radius)
    if results:
        third_circle, intersection1, intersection2 = results

        intermediate_circle = Curve.create_circle(
            center=SE2Transform([third_circle[0], third_circle[1]], theta=0),
            config_on_circle=SE2Transform([third_circle[0], third_circle[1]], theta=0),
            radius=radius,
            curve_type=DubinsSegmentType.LEFT,
        )

        for reverse_first_circle in [-1, 1]:
            for reverse_second_circle in [-1, 1]:
                for reverse_intermediate_circle in [-1, 1]:
                    gear1 = -1
                    gear2 = 1
                    angle1_rad, angle2_rad = theta_intersections(
                        third_circle, intersection1, intersection2, gear1, gear2
                    )

                    intersection_1_object = SE2Transform([intersection1[0], intersection1[1]], angle1_rad)
                    intersection_2_object = SE2Transform([intersection2[0], intersection2[1]], angle2_rad)

                    segment_path = path_creator_sheep_CCC_motion(
                        start_config,
                        end_config,
                        radius,
                        cicle_start,
                        circle_end,
                        intermediate_circle,
                        reverse_first_circle,
                        reverse_second_circle,
                        reverse_intermediate_circle,
                        intersection_1_object,
                        intersection_2_object,
                    )

                    obj_list = segment_path
                    if all(obj.gear == obj_list[0].gear for obj in obj_list):
                        original_path, original_length_path = segment_path_length_calculator(
                            segment_path, original_path, original_length_path
                        )

    print(original_path)
    for items in original_path:
        if items.type != 0:
            print(
                items.start_config,
                items.end_config,
                items.center,
                items.type,
                items.gear,
            )

    return original_path
