import networkx as nx
import logging
import numpy as np
import copy


class PathFinder:
    def __init__(self):
        pass

    def astar_planner(self,graph,agent_pos,target_pos):
      """
        plan a path in the graph providing an agent with a target pos
        :param map : the graph
        :param agent: the row,col position of the agent in the grid
        :param target: the row,col position of the target in the grid
        :return:
      """
      if agent_pos is not None and  target_pos is not None:
       if agent_pos != target_pos:
        agent_row,agent_col = agent_pos
        target_row,target_col = target_pos
        try:
          path = nx.astar_path(graph, (agent_row, agent_col), (target_row, target_col)) #,weight='weight'
          return path
        except (nx.NetworkXNoPath, nx.NodeNotFound) as e:
            return None
       else:
           return None
      else:
          return None


    def plan_path_to_target(self, G, agent_pos, target_pos, radius=5):
        """ Plan the shortest path from agent_pos to target_pos using A* on a subgraph around the agent position
        :param G: The graph to plan on
        :param agent_pos: The current position of the agent
        :param target_pos: The target position
        :param radius: The radius around the agent position to consider in the subgraph
        :returns: The shortest path from agent_pos to target_pos
        """
        subgraph = nx.ego_graph(G, agent_pos, radius=radius, center=True, undirected=True)
        path     = nx.astar_path(subgraph, agent_pos, target_pos, weight='weight')
        return path


    def astar_replan(self, graph, agent_pos, target_pos,neighbors_2_remove):
      """
        plan a path in the graph providing an agent with a target pos
        :param map : the graph
        :param agent: the row,col position of the agent in the grid
        :param target: the row,col position of the target in the grid
        :return:
      """
      if agent_pos is not None and target_pos is not None:
       if agent_pos != target_pos :
        G = graph.copy()

        if neighbors_2_remove is not None and len(neighbors_2_remove) > 0 :
          for node_pos in neighbors_2_remove:
              if node_pos in G.nodes and node_pos != agent_pos and node_pos != target_pos :
                 G.remove_node(node_pos)


        agent_row, agent_col   = agent_pos
        target_row, target_col = target_pos

        #n=[(agent_row+1, agent_col),(agent_row-1, agent_col),(agent_row, agent_col+1),(agent_row, agent_col-1)]

        try:
            path = nx.astar_path(G, (agent_row, agent_col), (target_row, target_col))#,weight='weight'
            #logging.info(f'New path found for : {agent_pos} -->{target_pos}!')
            return path

        except (nx.NetworkXNoPath, nx.NodeNotFound) as e:
            #logging.info(f'Sorry no path found for: {agent_pos} -->{target_pos}')
            return None

       else:
            return None
      else:
           return None