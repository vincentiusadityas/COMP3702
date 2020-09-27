import sys
from sokoban_map import SokobanMap

"""
Tester script.

Use this script to test whether your output files are valid solutions. You
should avoid modifying this file directly.

COMP3702 2019 Assignment 1 Support Code

Last updated by njc 11/08/19
"""


def main(arglist):
    """
    Test whether the given output file is a valid solution to the given map file.
    :param arglist: map file name, output file name
    """
    if len(arglist) != 2:
        print("Running this file tests whether the given output file is a valid solution to the given map file.")
        print("Note that this does not indicate whether the output file is optimal in the number of steps taken.")
        print("Usage: tester.py [map_file_name] [output_file_name]")
        return

    map_file = arglist[0]
    soln_file = arglist[1]

    game_map = SokobanMap(map_file)

    f = open(soln_file, 'r')
    moves = f.readline().split(',')

    # apply each move in sequence
    error_occurred = False
    for i in range(len(moves)):
        move = moves[i]
        ret = game_map.apply_move(move)
        if not ret:
            print("ERROR: Impossible move performed at step " + str(i))
            error_occurred = True

    if error_occurred:
        return

    if game_map.is_finished():
        print("Puzzle solved in " + str(len(moves)) + " steps!")
    else:
        num_solved = 0
        for tgt in game_map.tgt_positions:
            if tgt in game_map.box_positions:
                num_solved += 1
        print("Puzzle incomplete. " + str(num_solved) + " out of " +
              str(len(game_map.tgt_positions)) + " targets satisfied.")


if __name__ == '__main__':
    main(sys.argv[1:])