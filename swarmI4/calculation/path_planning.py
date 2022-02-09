import networkx as nx


class PathFinder:
    def __init__(self):
        pass

    def astar_planner(self,map,agent_pos,target_pos):
        """
        plan a path in the graph providing an agent with a target pos
        :param map : the graph
        :param agent: the row,col position of the agent in the grid
        :param target: the row,col position of the target in the grid
        :return:
        """
        agent_row,agent_col = agent_pos
        target_row,target_col = target_pos
        path = nx.astar_path(map, (agent_row, agent_col), (target_row, target_col))

        return path


