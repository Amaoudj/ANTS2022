""" Contains the world description for the swarm """
from typing import Tuple
import networkx as nx
import numpy as np
# from networkx.drawing.nx_agraph import graphviz_layout
import logging
from swarmI4.agent import AgentInterface

from .color import FREE, AGENT, OBSTACLE, TARGET, PATH


class Map(object):

    """ The world representation """

    def __init__(self, graph, number_of_nodes: Tuple[int, int], start_goal:tuple=(None,None)):

        logging.debug("Initializing the world")
        self._graph = graph
        self._swarm = None
        self._number_of_nodes = number_of_nodes
        self.custom_start_list,self.custom_goal_list = start_goal


        #Defining a list of dictionaries as message box for agents to leave data
        self.msg_box= {}
        self.new_paths_node = {}

    @property
    def size_xy(self) -> Tuple[int, int]:
        return self._number_of_nodes

    @property
    def size_x(self) -> int:
        return self._number_of_nodes[0]

    @property
    def size_y(self) -> int:
        return self._number_of_nodes[1]

    def connected(self, node: int) -> list:
        """ Get the corrected nodes to node

        :node: The node id
        :returns: list of node

        """

        return list(nx.neighbors(self._graph, node))

    def set_as_obstacle(self, node_pos:tuple or list):
        """
        set a note to be in obstacle space
        which means it will be isolated from
        it's neighbors
        """
        if type(node_pos) is list:
            for n in node_pos:
                self._graph.remove_node(n)
                self._graph.add_node(n)
                self._graph.nodes[n]["obstacle"] = True
                self._graph.nodes[n]["state"] = 'obstacle'
        else:
            self._graph.remove_node(node_pos)
            self._graph.add_node(node_pos)
            self._graph.nodes[node_pos]["obstacle"] = True
            self._graph.nodes[node_pos]["state"] = 'obstacle'

    def set_as_free(self,node_pos:tuple or list):
        """
        set a note to be in free space
        which means it will be connected with it's
        neighbors
        """
        if type(node_pos) is list:
            for node_pos_i in node_pos:
                self._graph.remove_node(node_pos_i)
                self._graph.add_node(node_pos_i)
                self._graph.nodes[node_pos_i]["obstacle"] = False
                self._graph.nodes[node_pos_i]["agent"] = None
                # for agent in swarm.agents:
                #     if node_pos_i in agent.targets_list:
                #         continue
                self._graph.nodes[node_pos_i]["state"] = 'free_space'
                for neighbor in self.get_neighbors(node_pos_i, diagonal=False):
                    if neighbor in self._graph.nodes:
                        self._graph.add_edge(node_pos_i, neighbor, weight=1)
        else:

            self._graph.nodes[node_pos]["obstacle"] = False
            self._graph.nodes[node_pos]["agent"] = None
            self._graph.nodes[node_pos]["state"] = 'free_space'
            for neighbor in self.get_neighbors(node_pos,diagonal=False):
                if neighbor in self._graph.nodes:
                    self._graph.add_edge(node_pos,neighbor,weight = 1)

    def get_neighbors(self, node_pos, diagonal: bool = False):
        """
        get the neighbors of a node
        """
        row, col = node_pos
        if diagonal:  # is the diagonal direction considered ?
            return self._graph.neighbors(node_pos)
        else:
            return [(row, col + 1),
                    (row, col - 1),
                    (row + 1, col),
                    (row - 1, col)]

    def get_random_free_node(self)->tuple:
        """
        return a node located in the free space
        :return: random node
        """
        random_node = None
        node_state = ''
        while node_state is not 'free_space' :
            random_node_id = np.random.choice(range(0,len(self._graph.nodes)-1))
            random_node = list(self._graph.nodes)[random_node_id]
            node_state = self._graph.nodes[random_node]["state"]
        return random_node

    def get_move_away_node(self, node1, threshold_node)->tuple:
        """
        return the nearest node for the node1 while note passing threshold_node
        :return: random node
        """
        row, col= node1
        row1, col1 = threshold_node
        _node = None
        move_backward=False
        node_state = ''
        if row ==row1 :  # in the same line
           _node = (row-1, col)
           node_state = self._graph.nodes[_node]["state"]
           if node_state is not 'free_space':
              _node = (row + 1 ,col)
              node_state = self._graph.nodes[_node]["state"]
              if node_state is not 'free_space':
                 move_backward = True
                 if col1>col: # search left side
                    _node = (row, col-1)
                 else:   # search right side
                  _node = (row, col + 1)

        elif col==col1: # the same col
           _node = (row , col- 1)
           node_state = self._graph.nodes[_node]["state"]
           if node_state is not 'free_space':
               _node = (row , col+ 1)
               node_state = self._graph.nodes[_node]["state"]
               if node_state is not 'free_space':
                  move_backward = True
                  if row1 > row:  # search Up-side
                      _node = (row - 1, col)
                  else:  # search down-side
                      _node = (row +1, col )

        x, y = _node

        return _node,move_backward

    def get_move_away_nearest_node(self, node1, threshold_node) -> tuple:
        """
        return the nearest node for the node1 while note passing threshold_node
        :return: random node
        """
        row, col = node1
        row1, col1 = threshold_node
        _node = None
        move_backward = False
        node_state = ''
        if row == row1:  # in the same line
            _node = (row - 1, col)
            node_state = self._graph.nodes[_node]["state"]
            if node_state is not 'free_space':
                _node = (row + 1, col)
                node_state = self._graph.nodes[_node]["state"]
                if node_state is not 'free_space':
                    move_backward = True
                    if col1 > col:  # search left side
                        _node = (row, col - 1)
                    else:  # search right side
                        _node = (row, col + 1)

        elif col == col1:  # the same col
            _node = (row, col - 1)
            node_state = self._graph.nodes[_node]["state"]
            if node_state is not 'free_space':
                _node = (row, col + 1)
                node_state = self._graph.nodes[_node]["state"]
                if node_state is not 'free_space':
                    move_backward = True
                    if row1 > row:  # search Up-side
                        _node = (row - 1, col)
                    else:  # search down-side
                        _node = (row + 1, col)

        x, y = _node
        if x>=self.size_x or y>=self.size_y:
            _node=threshold_node  #no solution, thus return the 'threshold_node'

        return _node, move_backward

    def get_move_away_free_node(self, node1, threshold_node)->tuple:
        """
        return the nearest node for the node1 while note passing threshold_node
        :return: random node
        """
        row, col= node1

        row1, col1 = threshold_node
        _node = None
        node_state = ''
        if row ==row1 :  # in the same line
           _node = (row-1, col)
           node_state = self._graph.nodes[_node]["state"]
           if node_state is not 'free_space':
              _node = (row + 1 ,col)
              node_state = self._graph.nodes[_node]["state"]
              if node_state is not 'free_space':
                  _node = None

        elif col==col1: # the same col
           _node = (row , col- 1)
           node_state = self._graph.nodes[_node]["state"]
           if node_state is not 'free_space':
               _node = (row , col+ 1)
               node_state = self._graph.nodes[_node]["state"]
               if node_state is not 'free_space':
                   _node = None

        return _node

    def get_nearest_free_node(self,node1, threshold_node)->tuple:
        """
        return the nearest node for the node1 while note passing threshold_node
        :return: random node
        """
        row, col= node1
        row1, col1 = threshold_node
        _node = None
        node_state = ''
        while node_state is not 'free_space':
          i = 1
          if row==row1 :  # in the same line
             if i==1 :
                _node = (row-1, col)
             if i==2:
                _node = (row +1 ,col)

             if i == 3:
               if col1>col: # search left side
                 _node = (row, col-1)
                 col-=1
               else:        # search right side
                 _node = (row, col + 1)
                 col += 1   # go
               i = 1

          elif col ==col1: # the same col
              if i == 1:
                  _node = (row , col- 1)
              if i == 2:
                  _node = (row , col+ 1)

              if i == 3:
                  if row1 > row:  # search Up-side
                      _node = (row - 1, col)
                      row -= 1
                  else:  # search down-side
                      _node = (row +1, col )
                      row += 1  # go
                  i = 1

          node_state = self._graph.nodes[_node]["state"]

        return _node

    def free_neighboring_node(self,pos,prohibited_nodes):
        """
        find a free node in the neighborhood
        """
        neighborhood = self.get_neighbors(pos,diagonal=False)
        for node in neighborhood:
            if node in prohibited_nodes:
                continue
            else:
                if node in self._graph.nodes and self._graph.nodes[node]["state"] == 'free_space':
                    if self._graph.nodes[node]["agent"] is not None:
                        return None
                    else:
                        return node

    def occupied(self, position: Tuple[int, int]):
        """
        Check if a node is occupied
        :position: node position
        """
        #if position in self._graph:
        if position[0] < 0 or position[0] >= self.size_x or position[1] < 0 or position[1] >= self.size_y:
            return True

        return "agent" not in self._graph.nodes[position] or self._graph.nodes[position]["agent"] is not None

    def move_agent(self, agent: AgentInterface, new_position: Tuple[int, int]):
        assert self._graph.nodes[agent.position]["agent"] == agent, \
            f"Error, agent is not currently located at {agent.position}"
        assert self._graph.nodes[new_position]["agent"] is None, \
            f"Error, location is not free {new_position}"

        self._graph.nodes[agent.position]["agent"] = None
        self._graph.nodes[new_position]["agent"] = agent
        agent.position = new_position




    def add_agent_to_map(self, agent: AgentInterface):
        assert self._graph.nodes[agent.position]["agent"] is None, \
            f"Trying to place an agent at a none free location {agent.position}"
        self._graph.nodes[agent.position]["agent"] = agent

    def number_of_nodes(self) -> int:
        """ Get the size of the world """
        return nx.number_of_nodes(self._graph)

    def cost(self, node_1, node_2) -> int:
        """ Get the cost between two nodes

        :node_1: Node id
        :node_2: NOde id
        :returns: The cost
        """
        return self._graph[node_1][node_2]["weight"]

    def get_agents_numbers(self, node):

        """ Get the number of agents and a given node
        :node: The node id
        :returns: The number of agents
        """
        assert 0 <= node <= nx.number_of_nodes(self._graph), "Get number of agents from a non existing node"
        ret = self._graph.nodes[node].get("agents")
        if ret is None:
            return 0

        return ret

    def update_value(self, node: int, key: str, value):
        """ Update a value on a node

        :node: The node id
        :key: The key
        :value: The value to assign

        """
        node = int(node)
        assert 0 <= node <= nx.number_of_nodes(self._graph), "Updating value on a non-existing node"
        self._graph.nodes[node][key] = value

    def set_as_path(self,path):
        """
        mark the nodes of a path with the path state
        :return:
        """
        for node in path:
            self._graph.nodes[node]["state"] = 'path'


    def view(self, block=True):
        """ Show the world

        """
        color = []

        for _, data in list(self._graph.nodes.data()):
            if "agent" in data:
                if data["agent"] is None:
                    c = FREE
                else:
                    c = AGENT
            else:
                if "obstacle" in data:
                    c = OBSTACLE
                else:
                    assert False, "This should never happen"

            color.append(c)

#        color = self._graph.nodes(data="agent", default=None)
#        color = [FREE if c is None else AGENT for _, c in color]

        pos = {n: (n[0] * 10, n[1] * 10) for n in nx.nodes(self._graph)}

        nodes_graph = nx.draw_networkx_nodes(self._graph, pos=pos, node_color=color,
                                             node_size=160, node_shape="s", linewidths=1.0)

        nodes_graph.set_edgecolor('black')

    @property
    def nodes(self):
        return self._graph.nodes

    @property
    def graph(self):
        return self._graph