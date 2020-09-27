import math
import random
import sys
import timeit

import numpy as np

from support.angle import Angle
from support.problem_spec import ProblemSpec
from support.robot_config import make_robot_config_from_ee1, write_robot_config_list_to_file, make_robot_config_from_ee2
from tester import test_self_collision, test_obstacle_collision, test_config_equality, test_environment_bounds

"""
Solver script.

COMP3702 2019 Assignment 2 Solver Code

BY: Vincentius Aditya Sundjaja
ID: 45610099

Last updated by njc 27/09/19
"""


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """

    def __init__(self, spec, config, parent=None):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.parent = parent
        self.neighbors = []
        # self.distance_to_end = self.calculate_distance_to_goal()

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def calculate_distance_to_goal(self):
        my_points = self.config.points
        end_points = self.spec.goal.points

        total = 0
        for i in range(len(my_points)):
            a = np.array(my_points[i])
            b = np.array(end_points[i])
            total += np.linalg.norm(a - b)

        return total

    def set_parent(self, parent):
        self.parent = parent

    def get_list(self):
        current_state = self
        while current_state.parent:
            yield current_state.parent
            current_state = current_state.parent

        output = list()
        for parent in current_state.parents():
            output.append(parent)

    def get_successors(self):
        return self.neighbors


def build_state_graph(spec, init_node, goal_node):
    configs = list()
    nodes = list()

    num_segments = spec.num_segments
    num_grapple_points = spec.num_grapple_points

    n = 1
    total_nodes = 100 * num_segments + 100 * (num_grapple_points - 1)

    nodes.append(init_node)

    while n <= total_nodes:
        new_angles = list()
        new_lengths = list()
        for i in range(num_segments):
            if i == 0:
                new_angles.append(Angle(degrees=float(random.uniform(-180, 180))))
            else:
                new_angles.append(Angle(degrees=float(random.uniform(-165, 165))))

            if spec.min_lengths[i] == spec.max_lengths[i]:
                new_lengths.append(spec.min_lengths[i])
            else:
                new_lengths.append(random.uniform(spec.min_lengths[i], spec.max_lengths[i]))

        if init_node.config.ee1_grappled:
            config = make_robot_config_from_ee1(init_node.config.get_ee1()[0], init_node.config.get_ee1()[1],
                                                new_angles, new_lengths, ee1_grappled=True)
        else:
            config = make_robot_config_from_ee2(init_node.config.get_ee2()[0], init_node.config.get_ee2()[1],
                                                new_angles, new_lengths, ee2_grappled=True)

        new_node = GraphNode(spec, config)

        if test_obstacle_collision(config, spec, spec.obstacles) and test_self_collision(config, spec) and \
                test_environment_bounds(config) and new_node not in nodes:
            nodes.append(new_node)
            configs.append(config)
            n = n + 1

    nodes.append(goal_node)

    angle_radius = 0.25 * num_segments + 0.05 * (num_grapple_points - 1)
    num = 1
    for node in nodes:
        nearest = find_nearest(nodes, node, angle_radius)

        for item in nearest:
            edge_splitter(item, node, spec)

        # print(num, ". NEIGHBOURS: ", len(nearest))
        num += 1

    print("NEIGHBOURS FOUND")


def find_bridge_config(spec, source, grapple_point, target):
    bridge_config_found = False
    num_segments = spec.num_segments

    bridge_config = None

    configs = list()

    while not bridge_config_found:
        new_angles = list()
        new_lengths = list()

        for i in range(num_segments - 1):
            if i == 0:
                new_angles.append(Angle(degrees=float(random.uniform(-180, 180))))
            else:
                new_angles.append(Angle(degrees=float(random.uniform(-165, 165))))

            if spec.min_lengths[i] == spec.max_lengths[i]:
                new_lengths.append(spec.min_lengths[i])
            else:
                new_lengths.append(random.uniform(spec.min_lengths[i], spec.max_lengths[i]))

        if source.ee1_grappled:
            possible_config = make_robot_config_from_ee1(source.get_ee1()[0], source.get_ee1()[1],
                                                         new_angles, new_lengths, source.ee1_grappled,
                                                         source.ee2_grappled)
            configs.append(possible_config)
        else:
            possible_config = make_robot_config_from_ee2(source.get_ee2()[0], source.get_ee2()[1],
                                                         new_angles, new_lengths, source.ee1_grappled,
                                                         source.ee2_grappled)
            configs.append(possible_config)

        end_points = possible_config.get_ee2() if possible_config.ee1_grappled else possible_config.get_ee1()
        line_length = calculate_distance(end_points, grapple_point)

        if spec.min_lengths[-1] <= line_length <= spec.max_lengths[-1]:
            second_last_pt = possible_config.points[-2]

            end_points_to_grapple = [x - y for x, y in zip(grapple_point, end_points)]
            end_points_to_second_last = [x - y for x, y in zip(second_last_pt, end_points)]

            arc_cos = Angle.acos(np.dot(end_points_to_grapple, end_points_to_second_last) /
                                 (np.linalg.norm(end_points_to_grapple) *
                                  np.linalg.norm(end_points_to_second_last)))

            if possible_config.ee1_grappled:
                last_angle = Angle(degrees=180) + arc_cos
            else:
                last_angle = Angle(degrees=180) - arc_cos

            if -165 < last_angle.in_degrees() < 165:

                new_lengths.append(line_length)
                new_angles.append(last_angle)

                if source.ee1_grappled:
                    possible_config = make_robot_config_from_ee1(source.get_ee1()[0], source.get_ee1()[1], new_angles,
                                                                 new_lengths, source.ee1_grappled, source.ee2_grappled)
                else:
                    possible_config = make_robot_config_from_ee2(source.get_ee2()[0], source.get_ee2()[1], new_angles,
                                                                 new_lengths, source.ee1_grappled, source.ee2_grappled)

                last_point = possible_config.get_ee2() if possible_config.ee1_grappled else possible_config.get_ee1()

                if test_obstacle_collision(possible_config, spec, spec.obstacles) and \
                        test_self_collision(possible_config, spec) and test_environment_bounds(possible_config) and \
                        last_point == grapple_point:
                    bridge_config = possible_config
                    bridge_config_found = True

    return bridge_config


def calculate_distance(p, q):
    dist = math.sqrt(((p[0] - q[0]) ** 2) + ((p[1] - q[1]) ** 2))
    return dist


def find_nearest(nodes, node, radius):
    near_list = []

    if node.config.ee1_grappled:
        angles = node.config.ee1_angles
    else:
        angles = node.config.ee2_angles

    for item in nodes:

        if item.config.ee1_grappled:
            item_angles = item.config.ee1_angles
        else:
            item_angles = item.config.ee2_angles

        res_list = list()

        for i in range(len(angles)):
            angle_1 = angles[i].in_degrees()
            angle_2 = item_angles[i].in_degrees()

            res_list.append(Angle(degrees=min(abs(angle_1 - angle_2), (360 - abs(angle_1 - angle_2)))))

        can_add = True

        if item in near_list or item is node:
            can_add = False

        else:
            for res in res_list:
                if abs(res.in_radians()) <= radius:
                    can_add = True
                else:
                    can_add = False
                    break

        if can_add:
            item.set_parent(node)
            near_list.append(item)

    return near_list


def edge_splitter(source, target, spec):
    # 15, 31, 63, ...
    total_split = 15
    ordered_list = list([None] * total_split)

    left_idx = 0
    right_idx = total_split-1

    mid = create_mid(source, target, spec)
    mid_idx = int(math.ceil((right_idx + left_idx) / 2))

    queue = [(source, target, mid, left_idx, right_idx, mid_idx)]

    ordered_list[mid_idx] = mid

    for i in range(int((total_split-1)/2)):
        left, right, mid, left_idx, right_idx, mid_idx = queue.pop(0)

        left_mid = create_mid(left, mid, spec)
        left_mid_idx = int(math.floor((left_idx + mid_idx) / 2))

        if ordered_list[left_mid_idx] is None:
            ordered_list[left_mid_idx] = left_mid

        queue.append((left, mid, left_mid, left_idx, mid_idx, left_mid_idx))

        mid_right = create_mid(mid, right, spec)
        mid_right_idx = int(math.ceil((mid_idx + right_idx) / 2))

        if ordered_list[mid_right_idx] is None:
            ordered_list[mid_right_idx] = mid_right

        queue.append((mid, right, mid_right, mid_idx, right_idx, mid_right_idx))

    source.neighbors.append(ordered_list[0])
    ordered_list[0].neighbors.append(source)

    j = 0
    for j in range(len(ordered_list) - 1):
        ordered_list[j].neighbors.append(ordered_list[j + 1])
        ordered_list[j + 1].neighbors.append(ordered_list[j])

    ordered_list[j + 1].neighbors.append(target)
    target.neighbors.append(ordered_list[j + 1])

    return True


def create_mid(source, target, spec):
    mid_angles = list()
    mid_lengths = [sum(x) / 2 for x in zip(source.config.lengths, target.config.lengths)]

    if source.config.ee1_grappled:
        for i in range(len(source.config.ee1_angles)):
            mid_angles.append(Angle(degrees=(source.config.ee1_angles[i].in_degrees() +
                                             target.config.ee1_angles[i].in_degrees()) / 2))

        mid_conf = make_robot_config_from_ee1(source.config.get_ee1()[0], source.config.get_ee1()[1],
                                              mid_angles, mid_lengths, ee1_grappled=True)
    else:
        for i in range(len(source.config.ee2_angles)):
            mid_angles.append(Angle(degrees=(source.config.ee2_angles[i].in_degrees() +
                                             target.config.ee2_angles[i].in_degrees()) / 2))

        mid_conf = make_robot_config_from_ee2(source.config.get_ee2()[0], source.config.get_ee2()[1],
                                              mid_angles, mid_lengths, ee2_grappled=True)

    return GraphNode(spec, mid_conf)


def solve(spec):
    """
    An example solve method containing code to perform a breadth first search of the state graph and return a list of
    configs which form a path through the state graph between the initial and the goal. Note that this path will not
    satisfy the primitive step requirement - you will need to interpolate between the configs in the returned list.

    If you wish to use this code, you may either copy this code into your own file or add your existing code to this
    file.

    :param spec: ProblemSpec object
    :return: List of configs forming a path through the graph from initial to goal
    """

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    if spec.num_grapple_points == 1:
        path = search_path(spec, init_node, goal_node)

        if path is None:
            return None

        else:
            configs = generate_primitive_steps(path)
            return configs

    elif spec.num_grapple_points == 2:

        bridge_config = find_bridge_config(spec, init_node.config, spec.grapple_points[-1], goal_node.config)
        bridge_node = GraphNode(spec, bridge_config)

        if bridge_config.ee1_grappled:
            next_bridge_config = make_robot_config_from_ee2(bridge_config.get_ee2()[0], bridge_config.get_ee2()[1],
                                                            bridge_config.ee2_angles, bridge_config.lengths,
                                                            ee1_grappled=False, ee2_grappled=True)

        else:
            next_bridge_config = make_robot_config_from_ee1(bridge_config.get_ee1()[0], bridge_config.get_ee1()[1],
                                                            bridge_config.ee1_angles, bridge_config.lengths,
                                                            ee1_grappled=False, ee2_grappled=True)

        next_bridge_node = GraphNode(spec, next_bridge_config)

        path_1 = search_path(spec, init_node, bridge_node)
        path_2 = search_path(spec, next_bridge_node, goal_node)

        if path_1 is None or path_2 is None:
            return None

        else:

            configs_1 = generate_primitive_steps(path_1)
            configs_2 = generate_primitive_steps(path_2)

            return configs_1 + configs_2

    elif spec.num_grapple_points > 2:

        bridge_configs = list()
        for i in range(spec.num_grapple_points - 2):
            max_distance_to_goal_grapple = calculate_distance(spec.grapple_points[i], spec.grapple_points[-1])
            max_length_from_current = sum(spec.max_lengths)

            if max_distance_to_goal_grapple > max_length_from_current:
                bridge_config = find_bridge_config(spec, init_node.config, spec.grapple_points[i+1], goal_node.config)
                bridge_configs.append(bridge_config)

                if bridge_config.ee1_grappled:
                    next_bridge_config = make_robot_config_from_ee2(bridge_config.get_ee2()[0],
                                                                    bridge_config.get_ee2()[1],
                                                                    bridge_config.ee2_angles, bridge_config.lengths,
                                                                    ee1_grappled=False, ee2_grappled=True)

                else:
                    next_bridge_config = make_robot_config_from_ee1(bridge_config.get_ee1()[0],
                                                                    bridge_config.get_ee1()[1],
                                                                    bridge_config.ee1_angles, bridge_config.lengths,
                                                                    ee1_grappled=False, ee2_grappled=True)
                bridge_configs.append(next_bridge_config)

        bridge_config = find_bridge_config(spec, bridge_configs[-1], spec.grapple_points[-1], goal_node.config)
        bridge_configs.append(bridge_config)

        write_robot_config_list_to_file("test_config.txt", [init_node.config] + bridge_configs + [goal_node.config])


def search_path(spec, init_node, goal_node):
    build_state_graph(spec, init_node, goal_node)
    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, goal_node.config, spec):
            # found path to goal
            print("PATH FOUND")
            return init_visited[current]

        successors = current.get_successors()

        for suc in successors:
            if suc not in init_visited:
                if test_self_collision(suc.config, spec) and test_environment_bounds(suc.config) and \
                        test_obstacle_collision(suc.config, spec, spec.obstacles):
                    init_container.append(suc)
                    init_visited[suc] = init_visited[current] + [suc.config]


def generate_primitive_steps(results):
    configs = list()
    configs.append(results[0])

    for i in range(len(results) - 1):
        angles_1 = results[i].ee1_angles if results[i].ee1_grappled else results[i].ee2_angles
        angles_2 = results[i + 1].ee1_angles if results[i + 1].ee1_grappled else results[i + 1].ee2_angles

        diff_angles = [x.in_radians() - y.in_radians() for x, y in zip(angles_2, angles_1)]

        lengths_1 = results[i].lengths
        lengths_2 = results[i + 1].lengths

        diff_lengths = [x - y for x, y in zip(lengths_2, lengths_1)]

        diff = diff_angles + diff_lengths
        max_diff = max([abs(x) for x in diff])

        n_steps = int(math.ceil(max_diff / 0.001))
        delta_angles = [x / n_steps for x in diff_angles]
        delta_lengths = [x / n_steps for x in diff_lengths]

        for j in range(n_steps):
            new_angles = [Angle(radians=x.in_radians()+(j*y)) for x, y in zip(angles_1, delta_angles)]
            new_lengths = [x+(j*y) for x, y in zip(lengths_1, delta_lengths)]

            if results[i].ee1_grappled:
                new_config = make_robot_config_from_ee1(results[i].get_ee1()[0], results[i].get_ee1()[1], new_angles,
                                                        new_lengths, results[i].ee1_grappled,
                                                        results[i].ee2_grappled)
            else:
                new_config = make_robot_config_from_ee2(results[i].get_ee2()[0], results[i].get_ee2()[1], new_angles,
                                                        new_lengths, results[i].ee1_grappled,
                                                        results[i].ee2_grappled)

            configs.append(new_config)

        configs.append(results[i + 1])

    return configs


def main(arglist):
    if len(arglist) == 0 or len(arglist) > 1:
        print("Running this file launches a program for generating solution to the input file.")
        print("Usage: solver.py [input_file]")
        return

    spec = ProblemSpec(arglist[0])

    start = timeit.default_timer()
    configs = solve(spec)

    if configs is None:
        print("NO SOLUTION FOUND")
    else:
        print("NUMBER OF STEPS: ", len(configs))
        write_robot_config_list_to_file("output.txt", configs)

    print('Time: ', timeit.default_timer() - start)


if __name__ == '__main__':
    main(sys.argv[1:])
