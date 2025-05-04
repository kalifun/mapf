import heapq
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def spacetime_astar(graph, start, goal, constraints=None):
    if constraints is None:
        constraints = set()
    else:
        constraints = set(constraints)  # 使用集合加快查找

    # 启发式函数：最短路径长度（不含时间）
    def heuristic(node):
        try:
            return nx.shortest_path_length(graph, node, goal)
        except nx.NetworkXNoPath:
            return float('inf')

    # 优先队列：(f值, 时间, 节点, 路径)
    pq = [(heuristic(start), 0, start, [start])]
    visited = set()  # 记录访问过的 (节点, 时间) 状态

    while pq:
        f, t, node, path = heapq.heappop(pq)
        state = (node, t)
        if state in visited:
            continue
        visited.add(state)

        if node == goal:
            return path

        # 扩展到邻居节点
        for neighbor in graph.neighbors(node):
            if (neighbor, t + 1) not in constraints:
                new_path = path + [neighbor]
                g = t + 1  # 当前时间步
                h = heuristic(neighbor)
                heapq.heappush(pq, (g + h, t + 1, neighbor, new_path))

        # 停留在当前节点（等待）
        if (node, t + 1) not in constraints:
            new_path = path + [node]
            g = t + 1
            h = heuristic(node)
            heapq.heappush(pq, (g + h, t + 1, node, new_path))

    return []  # 未找到路径


# 拓扑图类
class TopologyGraph:
    def __init__(self):
        self.graph = nx.Graph()
    
    def add_node(self, node):
        self.graph.add_node(node)
    
    def add_edge(self, node1, node2):
        self.graph.add_edge(node1, node2)

# 智能体类
class Agent:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.path = []

# A* 路径规划（简化为 networkx 的 shortest_path）
def astar(graph, start, goal, constraints=None):
    if constraints is None:
        constraints = []
    try:
        path = nx.shortest_path(graph, start, goal)
        # 检查路径是否满足约束
        for node, time in constraints:
            if len(path) > time and path[time] == node:
                return []  # 路径无效
        return path
    except nx.NetworkXNoPath:
        return []

# 冲突检测
def detect_conflicts(paths):
    max_len = max(len(path) for path in paths)
    conflicts = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            len_i = len(paths[i])
            len_j = len(paths[j])
            for t in range(max_len):
                pos_i = paths[i][t] if t < len_i else paths[i][-1]
                pos_j = paths[j][t] if t < len_j else paths[j][-1]
                if pos_i == pos_j:
                    conflicts.append((i, j, pos_i, t))
    return conflicts

# ECBS 算法类
class ECBS:
    def __init__(self, graph, agents, epsilon=0.5):
        self.graph = graph
        self.agents = agents
        self.epsilon = epsilon
        self.pos = nx.spring_layout(graph, seed=42)  # 固定布局以便观察
    
    def plan(self):
        # 初始路径规划
        for agent in self.agents:
            agent.path = spacetime_astar(self.graph, agent.start, agent.goal)
        
        # 检测并解决冲突
        conflicts = detect_conflicts([agent.path for agent in self.agents])
        if len(conflicts) != 0:
            print("路径冲突了 ", conflicts)
        max_attempts = 200
        attempt = 0
        
        while conflicts and attempt < max_attempts:
            conflict = conflicts[0]
            agent1_idx, agent2_idx, node, time = conflict
            
            # 为 agent1 添加约束
            constraints1 = [(node, time)]
            new_path1 = spacetime_astar(self.graph, self.agents[agent1_idx].start, self.agents[agent1_idx].goal, constraints1)
            
            if new_path1:
                self.agents[agent1_idx].path = new_path1
            else:
                # 为 agent2 添加约束
                constraints2 = [(node, time)]
                new_path2 = spacetime_astar(self.graph, self.agents[agent2_idx].start, self.agents[agent2_idx].goal, constraints2)
                if new_path2:
                    self.agents[agent2_idx].path = new_path2
                else:
                    print("No solution found after exploring constraints")
                    return []
            
            conflicts = detect_conflicts([agent.path for agent in self.agents])
            attempt += 1
        
        for agent in self.agents:
            if not agent.path:
                print("Failed to find a valid path for all agents")
                return []
        
        return [agent.path for agent in self.agents]
    def visualize(self, paths):
        fig, ax = plt.subplots(figsize=(8, 6))
        nx.draw(self.graph, self.pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10, ax=ax)
        
        # 初始化智能体位置
        agent_positions = [ax.plot([], [], 'o', markersize=15, label=f'Agent {i}')[0] for i in range(len(paths))]
        
        def update(frame):
            for i, agent_pos in enumerate(agent_positions):
                if frame < len(paths[i]):
                    x, y = self.pos[paths[i][frame]]
                    agent_pos.set_data([x], [y])
            return agent_positions
        
        # 创建动画
        if len(paths) == 0:
            return
        max_len = max(len(path) for path in paths if path)
        ani = FuncAnimation(fig, update, frames=range(max_len), interval=500, blit=True)
        ax.legend()
        plt.show()  # 显示动画
    
# 主程序
if __name__ == "__main__":
    # 创建拓扑图（类似道路网络）
    graph = TopologyGraph()
    for i in range(5):
        graph.add_node(i)
    graph.add_edge(0, 1)
    graph.add_edge(5, 1)
    graph.add_edge(6, 5)
    graph.add_edge(1, 2)
    graph.add_edge(2, 3)
    graph.add_edge(3, 4)
    graph.add_edge(1, 4)  # 共享路径，增加冲突可能性

    # 创建智能体
    agents = [Agent(0, 3), Agent(5,4)]

    # 运行 ECBS
    ecbs = ECBS(graph.graph, agents)
    paths = ecbs.plan()

    # 可视化路径和行走过程
    print("Agent paths:", paths)
    ecbs.visualize(paths)