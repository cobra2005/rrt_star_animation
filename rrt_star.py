import numpy as np
import matplotlib.pyplot as plt
import math
import random

# Node class for RRT*
class Node:
    def __init__(self, x, parent=None):
        self.x = x
        self.parent = parent
        self.cost = 0.0

# RRT* planner
def rrt_star(start, goal, obstacles, map_bounds, max_iter=500, step_size=0.5, search_radius=1.0):
    # Initialize tree
    nodes = [Node(start)]

    for i in range(max_iter):
        # Sample random point (with bias towards goal)
        if random.random() < 0.1:
            x_rand = goal
        else:
            x_rand = sample_free(map_bounds)

        # Find nearest node
        x_nearest = nearest(nodes, x_rand)
        # Steer toward x_rand
        x_new = steer(x_nearest.x, x_rand, step_size)

        # Check collision
        if not obstacle_free(x_nearest.x, x_new, obstacles):
            continue

        # Near nodes for RRT*
        X_near = near(nodes, x_new, search_radius)

        # Choose best parent
        node_min = x_nearest
        cost_min = x_nearest.cost + line_cost(x_nearest.x, x_new)
        for xn in X_near:
            if obstacle_free(xn.x, x_new, obstacles) and xn.cost + line_cost(xn.x, x_new) < cost_min:
                node_min = xn
                cost_min = xn.cost + line_cost(xn.x, x_new)

        # Create new node
        x_new_node = Node(x_new, parent=node_min)
        x_new_node.cost = cost_min
        nodes.append(x_new_node)

        # Rewire
        for xn in X_near:
            if obstacle_free(x_new, xn.x, obstacles) and x_new_node.cost + line_cost(x_new_node.x, xn.x) < xn.cost:
                xn.parent = x_new_node
                xn.cost = x_new_node.cost + line_cost(x_new_node.x, xn.x)

        # Check if goal reached
        if np.linalg.norm(np.array(x_new) - np.array(goal)) < step_size:
            goal_node = Node(goal, parent=x_new_node)
            goal_node.cost = x_new_node.cost + line_cost(x_new_node.x, goal)
            nodes.append(goal_node)
            return nodes, goal_node

    return nodes, None

# Helper functions

def sample_free(bounds):
    x = random.uniform(bounds[0], bounds[2])
    y = random.uniform(bounds[1], bounds[3])
    return (x, y)


def nearest(nodes, x_rand):
    dlist = [math.hypot(n.x[0] - x_rand[0], n.x[1] - x_rand[1]) for n in nodes]
    return nodes[dlist.index(min(dlist))]


def steer(x_from, x_to, step_size):
    vec = np.array(x_to) - np.array(x_from)
    dist = np.linalg.norm(vec)
    if dist < step_size:
        return x_to
    direction = vec / dist
    new_point = np.array(x_from) + step_size * direction
    return tuple(new_point)


def obstacle_free(p1, p2, obstacles):
    # Check line segment p1-p2 against circular obstacles
    for (ox, oy, r) in obstacles:
        # compute closest distance from center to segment
        d = dist_point_to_segment((ox, oy), p1, p2)
        if d <= r:
            return False
    return True


def dist_point_to_segment(p, a, b):
    # Return distance from point p to segment ab
    p = np.array(p); a = np.array(a); b = np.array(b)
    if np.all(a == b):
        return np.linalg.norm(p - a)
    t = np.dot(p - a, b - a) / np.dot(b - a, b - a)
    t = max(0, min(1, t))
    proj = a + t * (b - a)
    return np.linalg.norm(p - proj)


def near(nodes, x_new, radius):
    Xnear = []
    for n in nodes:
        if math.hypot(n.x[0] - x_new[0], n.x[1] - x_new[1]) <= radius:
            Xnear.append(n)
    return Xnear


def line_cost(x1, x2):
    return math.hypot(x1[0] - x2[0], x1[1] - x2[1])


def extract_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append(node.x)
        node = node.parent
    return path[::-1]

# Demo
if __name__ == '__main__':
    # Define environment
    start = (0, 0)
    goal = (10, 10)
    map_bounds = (-1, -1, 11, 11)
    # circular obstacles: (x, y, radius)
    obstacles = [
        (3, 5, 1.5),
        (7, 5, 1.5),
        (5, 8, 1.0)
    ]

    nodes, goal_node = rrt_star(start, goal, obstacles, map_bounds)

    # Plot
    plt.figure(figsize=(6,6))
    # Plot obstacles
    for (ox, oy, r) in obstacles:
        circle = plt.Circle((ox, oy), r, color='gray')
        plt.gca().add_patch(circle)
    # Plot tree
    for node in nodes:
        if node.parent:
            x1, y1 = node.x
            x2, y2 = node.parent.x
            plt.plot([x1, x2], [y1, y2], '-g')
    # Plot path
    if goal_node:
        path = extract_path(goal_node)
        xs, ys = zip(*path)
        plt.plot(xs, ys, '-r', linewidth=2)
    # Start and goal
    plt.plot(start[0], start[1], 'bo', markersize=8)
    plt.plot(goal[0], goal[1], 'ro', markersize=8)

    plt.xlim(map_bounds[0], map_bounds[2])
    plt.ylim(map_bounds[1], map_bounds[3])
    plt.title('RRT* Path Planning Demo')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()
