import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/utils")

from agent import AbstractSearchAgent
from plotting import Plotting
import generator as gn

class BFS_Agent(AbstractSearchAgent):
    def searching(self):
        start, goal = self.s_start, self.s_goal
        queue = [(start, [start])]
        visited = set()
        visited_order = []

        while queue:
            current, path = queue.pop(0)

            if current in visited:
                continue

            visited.add(current)
            visited_order.append(current)

            if current == goal:
                return path, visited_order

            for dx, dy in self.Env.motions:
                neighbor = (current[0] + dx, current[1] + dy)

                if neighbor not in visited and neighbor not in self.Env.obs:
                    queue.append((neighbor, path + [neighbor]))

        return [], visited_order  # If no path is found, return empty path

class DFS_Agent(AbstractSearchAgent):
    def searching(self):
        start, goal = self.s_start, self.s_goal
        stack = [(start, [start])]
        visited = set()
        visited_order = []

        while stack:
            current, path = stack.pop()

            if current in visited:
                continue

            visited.add(current)
            visited_order.append(current)

            if current == goal:
                return path, visited_order

            for dx, dy in self.Env.motions:
                neighbor = (current[0] + dx, current[1] + dy)

                if neighbor not in visited and neighbor not in self.Env.obs:
                    stack.append((neighbor, path + [neighbor]))

        return [], visited_order  # If no path is found, return empty path

class AStar_Agent(AbstractSearchAgent):
        def heuristic(self, node):
            return abs(node[0] - self.s_goal[0]) + abs(node[1] - self.s_goal[1])

        def searching(self):
            start, goal = self.s_start, self.s_goal
            open_list = [(0 + self.heuristic(start), 0, start, [start])]
            visited = set()
            visited_order = []

            while open_list:
                _, g, current, path = open_list.pop(0)

                if current in visited:
                    continue

                visited.add(current)
                visited_order.append(current)

                if current == goal:
                    return path, visited_order

                for dx, dy in self.Env.motions:
                    neighbor = (current[0] + dx, current[1] + dy)

                    if neighbor not in visited and neighbor not in self.Env.obs:
                        f = g + 1 + self.heuristic(neighbor)
                        open_list.append((f, g + 1, neighbor, path + [neighbor]))

                open_list.sort()

            return [], visited_order  # If no path is found, return empty path

if __name__ == "__main__":
    s_start = (5, 5) # Starting point
    s_goal = (45, 25) # Goal

    FPS = 60
    generate_mode = False # Turn to True to change the map
    map_name = 'bfs_astar'

    if generate_mode:
        gn.main(map_name)
    
    else:
        agent = AStar_Agent(s_start, s_goal, map_name) # Choose the agent here
        path, visited = agent.searching()

        # Plotting the path
        plot = Plotting(s_start, s_goal, map_name, FPS)

        plot.animation(path, visited)