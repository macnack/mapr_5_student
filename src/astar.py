#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import heapq as pq


class ASTAR(GridMap):
    def __init__(self):
        super(ASTAR, self).__init__()

    def heuristics(self, pos):
        distance = 0 # DELETE THIS LINE WHEN THE CODE INSERTED
        ### YOUR CODE GOES BELOW
        #
        #
        # IMPLEMENT HEURISTICS WITH MANHATTAN METRIC:
        # * follow the formula from the instruction
        #
        #
        ### YOUR CODE GOES ABOVE
        return distance

    def search(self):
        ### YOUR CODE GOES BELOW
        #
        #
        # IMPLEMENT A* SEARCH ALGORITHM:
        # * save your search in self.map.data
        # * use self.publish_visited() to publish the map every time you visited a new cell
        # * let 100 represent walls, 50 visited cells (useful for visualization)
        # * save the path to the goal fund by the algorithm to list of tuples: [(x_n, y_n), ..., (x_2, y_2), (x_1, y_1)]
        # * use self.publish_path(path) to publish the path at the very end
        # * start point is in self.start
        # * end point is in self.end
        #
        #
        ### YOUR CODE GOES ABOVE
        pass  # DELETE THIS LINE WHEN THE CODE INSERTED


if __name__ == '__main__':
    dfs = ASTAR()
    dfs.search()
