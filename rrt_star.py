import numpy as np
import matplotlib.pyplot as plt
import math
import random

# Node class for RRT*
class Node:
    def __init__(self, x, parent=None):
        self.x = x              # Tọa độ điểm
        self.parent = parent    # Nút cha trong cây
        self.cost = 0.0         # Chi phí từ gốc đến nút này

# RRT* planner

def rrt_star(start, goal, obstacles, map_bounds, max_iter=1000, step_size=0.5, search_radius=1.0):
    # Khởi tạo cây với nút gốc
    nodes = [Node(start)]
    best_goal_node = None  # Biến lưu node goal tốt nhất tìm được
    best_path = []
    best_path_line = None  # Đối tượng vẽ đường đi tốt nhất hiện tại

    plt.figure(1, figsize=(6, 6))
    plt.clf()

    # Vẽ chướng ngại vật
    for (ox, oy, r) in obstacles:
        circle = plt.Circle((ox, oy), r, color='gray')
        plt.gca().add_patch(circle)

    plt.plot(start[0], start[1], 'bo', markersize=8)
    plt.plot(goal[0], goal[1], 'ro', markersize=8)
    plt.xlim(map_bounds[0], map_bounds[2])
    plt.ylim(map_bounds[1], map_bounds[3])
    plt.grid(True)
    plt.title("RRT* Path Planning (Live)")

    for i in range(max_iter):
        print(f"Iteration: {i+1}")  # In ra số lần lặp hiện tại

        # Lấy mẫu ngẫu nhiên (bias về goal 10%)
        if random.random() < 0.1:
            x_rand = goal
        else:
            x_rand = sample_free(map_bounds)

        # Tìm nút gần nhất trong cây
        x_nearest = nearest(nodes, x_rand)
        # Tạo điểm mới theo hướng x_rand, cách x_nearest một đoạn step_size
        x_new = steer(x_nearest.x, x_rand, step_size)

        # Kiểm tra nếu không va chạm với chướng ngại vật
        if not obstacle_free(x_nearest.x, x_new, obstacles):
            continue

        # Tìm các nút lân cận trong bán kính search_radius
        X_near = near(nodes, x_new, search_radius)

        # Chọn cha tốt nhất (chi phí thấp nhất)
        node_min = x_nearest
        cost_min = x_nearest.cost + line_cost(x_nearest.x, x_new)
        for xn in X_near:
            if obstacle_free(xn.x, x_new, obstacles) and xn.cost + line_cost(xn.x, x_new) < cost_min:
                node_min = xn
                cost_min = xn.cost + line_cost(xn.x, x_new)

        # Tạo node mới và thêm vào cây
        x_new_node = Node(x_new, parent=node_min)
        x_new_node.cost = cost_min
        nodes.append(x_new_node)

        # Vẽ đoạn nối mới trên cây
        x1, y1 = x_new_node.x
        x2, y2 = node_min.x
        plt.plot([x1, x2], [y1, y2], '-g')
        plt.pause(0.001)  # cập nhật hình vẽ mỗi vòng lặp

        # Rewire: cập nhật lại các nút trong X_near nếu đi qua x_new rẻ hơn
        for xn in X_near:
            if obstacle_free(x_new, xn.x, obstacles) and x_new_node.cost + line_cost(x_new_node.x, xn.x) < xn.cost:
                xn.parent = x_new_node
                xn.cost = x_new_node.cost + line_cost(x_new_node.x, xn.x)

        # Nếu điểm mới gần goal thì luôn tạo đường đi và cập nhật nếu tốt hơn
        if np.linalg.norm(np.array(x_new) - np.array(goal)) < step_size:
            temp_goal = Node(goal, parent=x_new_node)
            temp_goal.cost = x_new_node.cost + line_cost(x_new_node.x, goal)
            nodes.append(temp_goal)

            if best_goal_node is None or temp_goal.cost < best_goal_node.cost:
                best_goal_node = temp_goal

            # Luôn vẽ đường đi tốt nhất hiện tại trong mỗi lần cập nhật goal
            path = extract_path(best_goal_node)
            if path != best_path:
                best_path = path
                xs, ys = zip(*path)
                # Xóa đường cũ nếu có
                if best_path_line:
                    best_path_line.remove()
                best_path_line, = plt.plot(xs, ys, '-r', linewidth=2)
                plt.pause(0.001)

    return nodes, best_goal_node

# Lấy mẫu ngẫu nhiên trong không gian cấu hình
def sample_free(bounds):
    x = random.uniform(bounds[0], bounds[2])
    y = random.uniform(bounds[1], bounds[3])
    return (x, y)

# Tìm node gần nhất trong danh sách nodes
def nearest(nodes, x_rand):
    dlist = [math.hypot(n.x[0] - x_rand[0], n.x[1] - x_rand[1]) for n in nodes]
    return nodes[dlist.index(min(dlist))]

# Di chuyển từ x_from về phía x_to một khoảng step_size
def steer(x_from, x_to, step_size):
    vec = np.array(x_to) - np.array(x_from)
    dist = np.linalg.norm(vec)
    if dist < step_size:
        return x_to
    direction = vec / dist
    new_point = np.array(x_from) + step_size * direction
    return tuple(new_point)

# Kiểm tra đoạn nối p1 - p2 có va chạm với bất kỳ chướng ngại vật nào không
def obstacle_free(p1, p2, obstacles):
    for (ox, oy, r) in obstacles:
        d = dist_point_to_segment((ox, oy), p1, p2)
        if d <= r:
            return False
    return True

# Tính khoảng cách từ điểm p đến đoạn thẳng a-b
def dist_point_to_segment(p, a, b):
    p = np.array(p); a = np.array(a); b = np.array(b)
    if np.all(a == b):
        return np.linalg.norm(p - a)
    t = np.dot(p - a, b - a) / np.dot(b - a, b - a)
    t = max(0, min(1, t))
    proj = a + t * (b - a)
    return np.linalg.norm(p - proj)

# Tìm các nút trong bán kính radius quanh x_new
def near(nodes, x_new, radius):
    Xnear = []
    for n in nodes:
        if math.hypot(n.x[0] - x_new[0], n.x[1] - x_new[1]) <= radius:
            Xnear.append(n)
    return Xnear

# Chi phí đoạn thẳng giữa 2 điểm
def line_cost(x1, x2):
    return math.hypot(x1[0] - x2[0], x1[1] - x2[1])

# Truy vết đường đi từ đích về gốc
def extract_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append(node.x)
        node = node.parent
    return path[::-1]

# Demo
if __name__ == '__main__':
    # Định nghĩa môi trường
    start = (0, 0)
    goal = (10, 10)
    map_bounds = (-1, -1, 11, 11)
    # Chướng ngại vật hình tròn: (x, y, bán kính)
    obstacles = [
        (3, 5, 1.5),
        (7, 5, 1.5),
        (5, 8, 1.0)
    ]

    # Chạy thuật toán
    nodes, goal_node = rrt_star(start, goal, obstacles, map_bounds)

    # Dừng lại để giữ hình ảnh sau khi thuật toán kết thúc
    plt.show()
