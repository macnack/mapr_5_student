import rclpy
import time
from mapr_5_student.grid_map import GridMap
import heapq
import math

class ASTAR(GridMap):
    def __init__(self):
        super(ASTAR, self).__init__('astar_node')

    def down(self):
        return (0, -1)

    def up(self):
        return (0, 1)

    def left(self):
        return (-1, 0)

    def right(self):
        return (1, 0)

    def idle(self):
        return (0, 0)

    def find_goal(self, node):
        return node == self.end

    def is_valid(self, pose):
        return pose[1] < self.map.info.width and pose[0] < self.map.info.height and pose[0] > 0 and pose[1] > 0

    def manhattan(self, pose):
        return abs(pose[0] - self.end[0]) + abs(pose[1] - self.end[1])
    
    def euclidean(self, pose):
        return math.sqrt( math.pow(pose[0] - self.end[0], 2) + math.pow(pose[1] - self.end[1], 2))

    def chebyshev(self, pose):
        return max( [ abs(pose[0] - self.end[0]), abs(pose[1] - self.end[1])])
    
    def cost_to_goal(self, pose):
        return self.chebyshev(pose)

    def cost_to_go(self, action):
        return 1.0

    def cost_final(self, pose, action):
        return self.heuristic(pose) + self.cost_to_go(action)

    def search(self):
        node_stack = []
        action = [self.down(), self.left(), self.up(), self.right()]
        cost_to_go = {self.start: 0}
        parent = {self.start: None}
        heapq.heappush(node_stack, (self.cost_to_goal(self.start), (self.start)))
        while node_stack:
            # Zabierz z kolejki element ktory ma najmniejsza wartosc
            _, cur_n = heapq.heappop(node_stack)
            # Sprawdz czy nie jest docelowym
            if self.find_goal(cur_n):
                break
            # Zaznacz jako odwiedzony
            self.map.data[cur_n[0] + cur_n[1] * self.map.info.width] = 50
            # Oblicz sąsiada
            neighbors = [(cur_n[0] + u[0], cur_n[1] + u[1]) for u in action]
            for next_n in neighbors:
                # Dla wszystkich sasiadow nie bedacych w Q
                if self.is_valid(next_n):
                    if self.map.data[next_n[0] + next_n[1] * self.map.info.width] < 100:
                        go_cost = cost_to_go[cur_n] + self.cost_to_go(next_n)
                        heuristic = self.cost_to_goal(next_n)
                        final_cost = go_cost + heuristic
                        if next_n not in cost_to_go or go_cost < cost_to_go[next_n]:
                            # Oblicz final gdy sasiad nie jest w odwiedzony lub jego koszt jest nizszy niz poprzedni
                            cost_to_go[next_n] = go_cost
                            parent[next_n] = cur_n
                            final_cost = go_cost + heuristic
                            heapq.heappush(node_stack, (final_cost, (next_n)))
            self.publish_visited()
        path = []
        cur_n = self.end
        while cur_n is not None:
            path.append(cur_n)
            cur_n = parent[cur_n]
        path.reverse()
        self.publish_path(path)


def main(args=None):
    rclpy.init(args=args)
    astar = ASTAR()
    while not astar.data_received():
        astar.get_logger().info("Waiting for data...")
        rclpy.spin_once(astar)
        time.sleep(0.5)

    astar.get_logger().info("Start graph searching!")
    astar.publish_visited()
    time.sleep(1)
    astar.search()


if __name__ == '__main__':
    main()
