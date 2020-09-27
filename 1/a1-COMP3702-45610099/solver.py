import copy
import sys
import queue
import operator
import timeit
import heapq

UP = (-1, 0)
DOWN = (1, 0)
LEFT = (0, -1)
RIGHT = (0, 1)

MOVE_DICT = {"u": UP,
             "d": DOWN,
             "l": LEFT,
             "r": RIGHT}

# input file symbols
BOX_SYMBOL = 'B'
TGT_SYMBOL = 'T'
PLAYER_SYMBOL = 'P'
OBSTACLE_SYMBOL = '#'
FREE_SPACE_SYMBOL = ' '
BOX_ON_TGT_SYMBOL = 'b'
PLAYER_ON_TGT_SYMBOL = 'p'

map_array = None

distance_to_targets = {}
deadlock_tiles = list()
all_positions = {}

box_positions = []
tgt_positions = []
player_position = tuple


def sokoban_map_parser(filename):
    f = open(filename, 'r')

    rows = []
    for line in f:
        if len(line.strip()) > 0:
            rows.append(list(line.strip()))

    f.close()

    row_len = len(rows[0])
    for row in rows:
        assert len(row) == row_len, "Mismatch in row length"

    num_rows = len(rows)

    global player_position

    for i in range(num_rows):
        for j in range(row_len):
            if rows[i][j] == BOX_SYMBOL:
                box_positions.append((i, j))
                rows[i][j] = FREE_SPACE_SYMBOL
                all_positions[(i, j)] = float('inf')
            elif rows[i][j] == TGT_SYMBOL:
                tgt_positions.append((i, j))
                rows[i][j] = FREE_SPACE_SYMBOL
                all_positions[(i, j)] = float('inf')
            elif rows[i][j] == PLAYER_SYMBOL:
                player_position = (i, j)
                rows[i][j] = FREE_SPACE_SYMBOL
                all_positions[(i, j)] = float('inf')
            elif rows[i][j] == BOX_ON_TGT_SYMBOL:
                box_positions.append((i, j))
                tgt_positions.append((i, j))
                rows[i][j] = FREE_SPACE_SYMBOL
                all_positions[(i, j)] = float('inf')
            elif rows[i][j] == PLAYER_ON_TGT_SYMBOL:
                player_position = (i, j)
                tgt_positions.append((i, j))
                rows[i][j] = FREE_SPACE_SYMBOL
                all_positions[(i, j)] = float('inf')
            elif rows[i][j] == FREE_SPACE_SYMBOL:
                all_positions[(i, j)] = float('inf')

    assert len(box_positions) == len(tgt_positions), "Number of boxes does not match number of targets"

    return rows


# Calculate distance from all possible positions to all targets
def get_target_pull_distance(state, targets):

    for target in targets:
        positions_dict = copy.deepcopy(all_positions)
        positions_dict[target] = 0
        distance_to_targets[target] = positions_dict

        q = queue.Queue()
        q.put(target)

        while not q.empty():
            position = q.get()

            for key, value in MOVE_DICT.items():
                box_pos = tuple(map(operator.add, position, value))
                player_pos = tuple(map(operator.add, position, tuple(2*x for x in value)))

                if state[box_pos[0]][box_pos[1]] != "#" and \
                        distance_to_targets[target][(box_pos[0], box_pos[1])] == float('inf'):

                    if state[player_pos[0]][player_pos[1]] != "#":

                        distance_to_targets[target][(box_pos[0], box_pos[1])] = \
                            distance_to_targets[target][position] + 1
                        q.put(box_pos)
                    else:
                        continue
                else:
                    continue

    # for key, value in distance_to_targets.items():
    #     print(key, ": ", value)
    #     print()


def get_deadlock_tiles():
    not_deadlock = list()

    for positions in distance_to_targets.values():
        for key, value in positions.items():
            if value == float('inf') and key not in deadlock_tiles and key not in not_deadlock:
                deadlock_tiles.append(key)
            elif value != float('inf') and key not in not_deadlock:
                not_deadlock.append(key)

                if key in deadlock_tiles:
                    deadlock_tiles.remove(key)

    # print(deadlock_tiles)


# Returns the non-tangential distance between two points
def manhattan_dist(t0, t1):
    return abs(t0[0] - t1[0]) + abs(t0[1] - t1[1])


class Sokoban:

    def __init__(self, search_type=None, player=None, boxes=None, parent=None, action=None, action_list='', cost=0):
        self.search_type = search_type
        self.parent = parent
        self.children = list()
        self.action = action
        self.action_list = action_list
        self.cost = cost

        # THIS IS FIRST DONE OUTSIDE THE CLASS AND MAY CHANGE FOR EACH STATE
        self.player = player    # tuple, eg: (1,2)
        self.boxes = boxes

        if self.boxes is not None:
            self.boxes.sort()

        # THIS IS DONE OUTSIDE THE CLASS SINCE ALL STATE WILL HAVE THE SAME PROPERTIES
        self.state = map_array  # map array with only the walls
        self.targets = tgt_positions  # list of tuple, [(1,2), (3,4),...]
        self.target_pull_distance = distance_to_targets  # distance to targets from all spaces except walls

    def is_goal(self):
        is_goal = True
        for i in self.boxes:
            if i not in self.targets:
                is_goal = False
        return is_goal

    def move(self):
        for key, value in MOVE_DICT.items():

            boxes = copy.deepcopy(self.boxes)

            p_next_row = self.player[0] + value[0]
            p_next_col = self.player[1] + value[1]

            action = key
            # r_action = REVERSED_MOVE_DICT[action]

            if self.state[p_next_row][p_next_col] == OBSTACLE_SYMBOL:
                continue
            else:
                new_player_position = (p_next_row, p_next_col)

                b_next_row = p_next_row + value[0]
                b_next_col = p_next_col + value[1]

                if new_player_position in boxes:
                    if self.state[b_next_row][b_next_col] == OBSTACLE_SYMBOL or \
                            (b_next_row, b_next_col) in boxes or \
                            (b_next_row, b_next_col) in deadlock_tiles:
                        continue
                    else:
                        new_box_position = (b_next_row, b_next_col)

                        # update box position
                        boxes.remove(new_player_position)
                        boxes.append(new_box_position)

            action_list = self.action_list + "," + action
            self.children.append(Sokoban(self.search_type, new_player_position, boxes,
                                         self, action, action_list, self.cost + 1))

        return self.children

    def get_heuristic(self):

        total = 0
        player_min_distance = float('inf')

        for box in self.boxes:

            min_box_distance = float('inf')
            for target in self.targets:
                box_distance = self.target_pull_distance[target][box]
                if box_distance < min_box_distance:
                    min_box_distance = box_distance

            total += min_box_distance

            player_distance = manhattan_dist(self.player, box)
            if player_distance < player_min_distance:
                player_min_distance = player_distance

        total += player_min_distance

        return total

    def get_parents(self):
        current_state = self
        while current_state.parent:
            yield current_state.parent
            current_state = current_state.parent

    def actions(self):
        current_state = self
        while current_state.parent:
            yield current_state.action
            current_state = current_state.parent

    def get_children(self):
        self.move()
        return self.children

    def get_state(self):
        return self.state

    def total_cost(self):
        return self.cost + self.get_heuristic()

    def __lt__(self, other):
        if self.search_type == "ucs":
            return self.cost < other.cost
        elif self.search_type == "a_star":
            return self.total_cost() < other.total_cost()

    def __eq__(self, other):
        return self.player == other.player and self.boxes == other.boxes

    def __hash__(self):
        return hash((self.player, tuple(self.boxes)))


def ucs(sokoban):
    total_node = 1

    container = []
    visited = set()

    item = (sokoban.cost, sokoban)
    heapq.heappush(container, item)

    if sokoban.is_goal():
        print("UCS DONE")
        print("Generated: ", total_node)
        print("In Fringe: ", len(container))
        print("Visited: ", len(visited))
        return sokoban

    while container:

        sokoban = heapq.heappop(container)[1]

        if sokoban not in visited:
            visited.add(sokoban)
        else:
            continue

        for child in sokoban.get_children():
            total_node += 1
            item = (child.cost, child)

            if child.is_goal():
                print("UCS DONE")
                print("Generated: ", total_node)
                print("In Fringe: ", len(container))
                print("Visited: ", len(visited))
                return child

            if child not in visited:
                heapq.heappush(container, item)
                # visited.add(child)

    return None


def a_star(sokoban):
    total_node = 1

    container = []
    visited = set()

    item = (sokoban.total_cost(), sokoban)
    heapq.heappush(container, item)

    if sokoban.is_goal():
        print("A* DONE")
        print("Generated: ", total_node)
        print("In Fringe: ", len(container))
        print("Visited: ", len(visited))
        return sokoban

    while container:

        sokoban = heapq.heappop(container)[1]

        if sokoban not in visited:
            visited.add(sokoban)
        else:
            continue

        for child in sokoban.get_children():
            total_node += 1
            item = (child.total_cost(), child)

            if child.is_goal():
                print("A* DONE")
                print("Generated: ", total_node)
                print("In Fringe: ", len(container))
                print("Visited: ", len(visited))
                return child

            if child not in visited:
                heapq.heappush(container, item)
                # visited.add(child)

    return None


def print_solution(current_state, out_file):

    if current_state is None:
        print("ERROR, no solution")
        return

    actions = current_state.action_list[1:]
    steps = current_state.cost

    if out_file:
        f = open(out_file, "w")
        f.write(actions)

    print(steps, "steps:", actions)


def main(arglist):

    if len(arglist) < 1 or len(arglist) > 2:
        print("Running this file directly launches a search algorithm of Sokoban based on the given map file.")
        print("Usage: sokoban_map.py [map_file_name] [output_file_name]")
        print("Output will be a set of actions that shows the shortest path to the goal printed on the stdout and "
              "the output file")
        return

    global map_array
    map_array = sokoban_map_parser(arglist[0])

    get_target_pull_distance(map_array, tgt_positions)
    get_deadlock_tiles()

    # #######-UCS-########-UCS-########-UCS-########-UCS-########-UCS-########
    start = timeit.default_timer()

    sokoban = Sokoban("ucs", player_position, box_positions)
    result = ucs(sokoban)

    print('Time: ', timeit.default_timer() - start)

    if len(arglist) > 1:
        print_solution(result, arglist[1])
    else:
        print_solution(result, None)

    print()
    # #######-A_STAR-#######-A_STAR-#######-A_STAR-#######-A_STAR-#######-A_STAR-#######
    start = timeit.default_timer()

    sokoban2 = Sokoban("a_star", player_position, box_positions)
    result2 = a_star(sokoban2)

    print('Time: ', timeit.default_timer() - start)

    if len(arglist) > 1:
        print_solution(result2, arglist[1])
    else:
        print_solution(result2, None)


if __name__ == '__main__':
    main(sys.argv[1:])
