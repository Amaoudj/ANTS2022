""" Contains the world description for the swarm """
from typing import Tuple
import networkx as nx
import numpy as np
# from networkx.drawing.nx_agraph import graphviz_layout
import logging
from swarmI4.agent import AgentInterface
import  random
from .color import FREE, AGENT, OBSTACLE, TARGET, PATH
from typing import List
#from quadtree import QuadTree

class Map(object):

    """ The world representation """

    def __init__(self, graph,copy_graph,
                number_of_nodes: Tuple[int, int],
                start_goal:tuple=(None,None),
                number_of_obstacles =0,
                number_of_targets =0,
                pattern_file_path=None):

        #logging.debug("Initializing the world")
        self._graph = graph
        self._copy_graph = copy_graph  # graph
        self._swarm = None

        self._number_of_nodes = number_of_nodes
        self.custom_start_list,self.custom_goal_list = start_goal
        self.pattern_file_path = pattern_file_path
        self.number_of_obstacles = number_of_obstacles
        self.number_of_targets = number_of_targets
        self.new_OBS = []
        #Defining a list of dictionaries as message box for agents to leave data
        #self.msg_box= {}
        self.new_paths_node = {}
        self.num_agents = 0
        self.neighbors_agents_stat= None

        # Update agent 0's stats
        #agent_stats[0] = {'next_node': (1, 2), 'moving_backward': False, ...}

        # Get agent 1's next node
       # next_node = agent_stats[1]['next_node']

    @property
    def size_xy(self) -> Tuple[int, int]:
        return self._number_of_nodes

    @property
    def size_x(self) -> int:
        return self._number_of_nodes[0]

    @property
    def size_y(self) -> int:
        return self._number_of_nodes[1]


    #def get_agents_nearby(agent, map, radius) -> List[dict]:
    #    """
    #    Get all agents within a given radius of the current agent.
    #    """
    #    nearby_agents = []
    #    qt = QuadTree(map.width, map.height)
    #    for a in map.agents_stat:
    #        if a['AgentID'] != agent.id:
    #            qt.insert(a['pos'], a)
    #    agents_in_radius = qt.query_radius(agent.position, radius)
    #    for a in agents_in_radius:
    #        nearby_agents.append(a.data)
    #    return nearby_agents

    def within_map_size(self, node):
            Toreturn = True
            x, y = node
            if x < 0 or x >= self.size_x or y < 0 or y >= self.size_y:
                ToReturn = False

            return Toreturn

    def occupied(self, position: Tuple[int, int]):
            """
            Check if a node is occupied
            :position: node position
            """
            # if position in self._graph:
            ToReturn = False
            x, y = position
            if x < 0 or x >= self.size_x or y < 0 or y >= self.size_y or self._graph.nodes[position]["agent"] is not None or self._graph.nodes[position]["obstacle"]:
                ToReturn = True

            return ToReturn

    def is_target(self, node):

        ToReturn = False

        if self._graph.nodes[node]["state"]=="target":
            ToReturn = True

        return ToReturn

    def is_obstacle(self, node):

            ToReturn = False
            x, y = node
            if x < 0 or x >= self.size_x or y < 0 or y >= self.size_y or self._graph.nodes[node]["obstacle"]:
                ToReturn = True

            return ToReturn

    def is_free(self, node):

            ToReturn = False

            if not self.occupied(node):
                if (self._graph.nodes[node]["state"] == 'free_space' or self._graph.nodes[node]["state"] == 'target' or
                        self._graph.nodes[node]["state"] == 'path'):
                    ToReturn = True

            return ToReturn

    def connected(self, node: int) -> list:
            """ Get the corrected nodes to node

            :node: The node id
            :returns: list of node
            """
            return list(nx.neighbors(self._graph, node))


    def get_nearest_free_node(self,mypos):
        """
          the function keeps track of checked neighbors using a set called checked_neighbors.
          If no free neighboring node is found in the current neighborhood, it updates the neighborhood set to include the
          neighbors of the current neighbors. The loop continues until a free node is found or all neighbors have been checked
        """
        _node = None
        neighborhood = self.get_neighbors(mypos, diagonal=False)
        checked_neighbors = set()
        num_tries = 0
        while _node is None and len(checked_neighbors) < len(neighborhood)and num_tries < 100:
            num_tries += 1
            for neighbor in neighborhood:
                if neighbor not in checked_neighbors:
                    _node = self.free_neighboring_node(neighbor, mypos)
                    if _node is not None and not self.is_obstacle(_node):
                        return _node
                    checked_neighbors.add(neighbor)

            # Update neighborhood to include neighbors of the current neighbors
            # i.e., expand the search for a free neighboring node to the next level of neighbors if the current level does not contain any free nodes.
            new_neighborhood = set()
            for neighbor in neighborhood:
                new_neighborhood.update(self.get_neighbors(neighbor, diagonal=False))
            neighborhood = new_neighborhood

        return _node

    def add_agent_to_map(self, agent: AgentInterface):
            assert self._graph.nodes[agent.position]["agent"] is None, \
                f"Trying to place an agent at a none free location {agent.position}"

            self._graph.nodes[agent.position]["agent"] = agent
            self._graph.nodes[agent.position]["obstacle"] = False
            self._graph.nodes[agent.position]["state"] = 'agent'

    def add_agent_to_list_agents_done(self, pos_agent):

            self.nodes_occupied_agents.append(pos_agent)
            for pos in self.nodes_occupied_agents:
                neighbors = self.get_neighbors(pos, diagonal=False)
                numer_free_neighbor = self.get_number_of_free_neighbors(neighbors)
                if numer_free_neighbor == 1:
                    self._copy_graph.remove_node(pos)

    def get_number_of_free_neighbors(self, neighbors):
            num = 0
            for node in neighbors:
                if self.is_free(node):
                    num += 1

            return num

    def set_as_obstacle(self, node_pos: tuple or list):
            """
            set a note to be in obstacle space
            which means it will be isolated from
            it's neighbors
            """
            if type(node_pos) is list:
                for n in node_pos:
                    self._graph.remove_node(n)
                    if n in self._copy_graph:
                        self._copy_graph.remove_node(n)
                    self._graph.add_node(n)
                    self._graph.nodes[n]["obstacle"] = True
                    self._graph.nodes[n]["state"] = 'obstacle'
                    self._graph.nodes[n]["agent"] = None
                    ###############################################################
                    # x,y=n
                    # N = [(x, y + 1), (x, y - 1), (x + 1, y), (x - 1, y)]
                    # for n in N:
                    #    if n in self._graph.nodes:
                    #        self._graph.add_edge((x, y), n, weight=1000000)
                    ###############################################################
            else:


                if node_pos in self._copy_graph:
                    self._copy_graph.remove_node(node_pos)

                self._graph.remove_node(node_pos)
                self._graph.add_node(node_pos)
                self._graph.nodes[node_pos]["obstacle"] = True
                self._graph.nodes[node_pos]["state"] = 'obstacle'
                self._graph.nodes[node_pos]["agent"] = None


    def set_as_free(self, node_pos: tuple or list):
            """
            set a note to be in free space
            which means it will be connected with it's
            neighbors
            """

            if type(node_pos) is list:
                for node_pos_i in node_pos:
                    self._graph.remove_node(node_pos_i)
                    self._graph.add_node(node_pos_i)
                    neighbors = self.get_neighbors(node_pos_i, diagonal=False)

                    if node_pos_i not in self._copy_graph:
                        self._copy_graph.add_node(node_pos_i)
                    else:
                        self._copy_graph.remove_node(node_pos_i)
                        self._copy_graph.add_node(node_pos_i)


                    for neighbor in neighbors:
                            if neighbor in self._copy_graph.nodes and not self.is_obstacle(neighbor):
                                self._copy_graph.add_edge(node_pos_i, neighbor, weight=1)

                    self._graph.nodes[node_pos_i]["obstacle"] = False
                    self._graph.nodes[node_pos_i]["agent"] = None
                    self._graph.nodes[node_pos_i]["state"] = 'free_space'

                    for neighbor in neighbors :
                        if neighbor in self._graph.nodes and not self.is_obstacle(neighbor):
                            self._graph.add_edge(node_pos_i, neighbor, weight=1)

            else:
                neighbors=self.get_neighbors(node_pos, diagonal=False)
                self._graph.remove_node(node_pos)
                self._graph.add_node(node_pos)

                self._graph.nodes[node_pos]["obstacle"] = False
                self._graph.nodes[node_pos]["agent"] = None
                self._graph.nodes[node_pos]["state"] = "free_space"

                if node_pos not in self._copy_graph:
                    self._copy_graph.add_node(node_pos)
                else:
                    self._copy_graph.remove_node(node_pos)
                    self._copy_graph.add_node(node_pos)


                for neighbor in neighbors:
                     if neighbor in self._copy_graph.nodes :
                       if not self.is_obstacle(neighbor) :
                          self._copy_graph.add_edge(node_pos, neighbor, weight=1)

                for neighbor in neighbors:
                   if neighbor in self._graph.nodes :
                     if not self.is_obstacle(neighbor):
                        self._graph.add_edge(node_pos, neighbor, weight=1)


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

    def get_random_free_node(self) -> tuple:
            """
            return a node located in the free space
            :return: random node
            """
            random_node = None
            node_state = ''
            num_tries = 0
            while node_state != 'free_space' and num_tries< 100 :
                num_tries += 1
                random_node_id = np.random.choice(range(0, len(self._graph.nodes) - 1))
                random_node = list(self._graph.nodes)[random_node_id]
                node_state = self._graph.nodes[random_node]["state"]
            return random_node

    def get_random_OBS(self) -> tuple:

            """
            return a node located in the free space
            :return: random node
            """
            random_node = None
            num_tries=0
            node_state = ''
            while node_state != 'obstacle' and num_tries < 50 :
                num_tries += 1
                random_node_id = np.random.choice(range(0, len(self._graph.nodes) - 1))
                random_node = list(self._graph.nodes)[random_node_id]
                node_state = self._graph.nodes[random_node]["state"]
            return random_node

    def get_WayNode_include_moveBackward(self, node1, threshold_node) -> tuple:
         """
            return the nearest node for the node1 while note passing threshold_node
            :return: random node
         """

         _node = None
         move_backward = False

         if node1 is not None and threshold_node is not None:
            row, col = node1
            row1, col1 = threshold_node


            if row == row1:  # in the same line
                _node1 = (row - 1, col)
                _node2 = (row + 1, col)

                if self.is_free(_node1):  # and not self.is_free(_node2):
                    _node = _node1
                elif self.is_free(_node2):  # and not self.is_free(_node1):
                    _node = _node2

                else:
                    move_backward = True
                    if col1 > col:  # search left side
                        _node = (row, col - 1)
                        if not self.within_map_size(_node) or self.is_obstacle(_node):
                            _node = None#node1
                            move_backward = False
                    else:  # search right side
                        _node = (row, col + 1)
                        if not self.within_map_size(_node) or self.is_obstacle(_node):
                            _node = None#node1
                            move_backward = False

            elif col == col1:  # the same col

                _node1 = (row, col - 1)
                _node2 = (row, col + 1)

                if self.is_free(_node1):  # and not self.is_free(_node2):
                    _node = _node1
                elif self.is_free(_node2):  # and not self.is_free(_node1):
                    _node = _node2

                else:

                    move_backward = True
                    if row1 > row:  # search Up-side

                        _node = (row - 1, col)
                        if not self.within_map_size(_node) or self.is_obstacle(_node):
                            _node = None#node1
                            move_backward = False

                    else:  # search down-side
                        _node = (row + 1, col)
                        if not self.within_map_size(_node) or self.is_obstacle(_node):
                            _node = None#node1
                            move_backward = False

         return _node, move_backward

    def get_Free_WayNode(self, node1, threshold_node, prohibited_node) -> tuple:
         """
            return the nearest node for the node1 while note passing threshold_node
            :return: random node
         """
         _node = None
         move_backward = False

         if  node1 is not None and threshold_node is not None:
            row, col = node1
            row1, col1 = threshold_node


            if prohibited_node is not None:
                if row == row1:  # in the same line
                    _node1 = (row - 1, col)  # up
                    _node2 = (row + 1, col)  # down

                    if self.is_free(_node1) and _node1 != prohibited_node:  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.is_free(_node2) and _node2 != prohibited_node:  # and not self.is_free(_node1):
                        _node = _node2

                    else:

                        if col1 > col:  # search left side
                            if self.is_free((row, col - 1)) and (row, col - 1) != prohibited_node:  # self._graph.nodes[(row, col - 1)]["state"] == 'free_space' or self._graph.nodes[(row, col - 1)]["state"] == 'target':
                                _node = (row, col - 1)
                                move_backward = True

                        else:  # search right side
                            if self.is_free((row, col + 1)) and (row, col + 1) != prohibited_node:  # self._graph.nodes[(row, col + 1)]["state"] == 'free_space' or self._graph.nodes[(row, col + 1)]["state"] == 'target':
                                _node = (row, col + 1)
                                move_backward = True

                elif col == col1:  # the same col

                    _node1 = (row, col - 1)
                    _node2 = (row, col + 1)

                    if self.is_free(_node1) and _node1 != prohibited_node:  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.is_free(_node2) and _node2 != prohibited_node:  # and not self.is_free(_node1):
                        _node = _node2
                    else:

                        if row1 > row:  # search Up-side
                            if self.is_free((row - 1, col)) and (row - 1, col) != prohibited_node:  # self._graph.nodes[(row- 1, col )]["state"] == 'free_space' or self._graph.nodes[(row- 1, col )]["state"] == 'target':########################
                                _node = (row - 1, col)
                                move_backward = True
                        else:  # search down-side
                            if self.is_free((row + 1, col)) and (row + 1,col) != prohibited_node:  # self._graph.nodes[(row + 1, col)]["state"] == 'free_space' or self._graph.nodes[(row + 1, col)]["state"] == 'target':  ########################
                                _node = (row + 1, col)
                                move_backward = True

         return _node, move_backward


    def get_right_or_left_free_node(self, node1, threshold_node,prohibited_node) -> tuple:  # search only in two sides
          """
            return the nearest node for the node1 while note passing threshold_node
            :return: random node
          """
          _node = None
          if node1 is not None and threshold_node is not None:
            row, col = node1

            row1, col1 = threshold_node

            if prohibited_node is None:
                if row == row1:  # in the same line
                    _node1 = (row - 1, col)
                    _node2 = (row + 1, col)

                    if self.is_free(_node1):  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.is_free(_node2):  # and not self.is_free(_node1):
                        _node = _node2
                    else:
                        _node = None

                elif col == col1:  # the same col
                    _node1 = (row, col - 1)
                    _node2 = (row, col + 1)

                    if self.is_free(_node1):  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.is_free(_node2):  # and not self.is_free(_node1):
                        _node = _node2
                    else:
                        _node = None
            else:

                if row == row1:  # in the same line
                    _node1 = (row - 1, col)
                    _node2 = (row + 1, col)

                    if self.is_free(_node1) and _node1 != prohibited_node:  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.is_free(_node2) and _node2 != prohibited_node:  # and not self.is_free(_node1):
                        _node = _node2
                    else:
                        _node = None

                elif col == col1:  # the same col
                    _node1 = (row, col - 1)
                    _node2 = (row, col + 1)

                    if self.is_free(_node1) and _node1 != prohibited_node:  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.is_free(_node2) and _node2 != prohibited_node:  # and not self.is_free(_node1):
                        _node = _node2
                    else:
                        _node = None

          return _node

    def get_right_or_left_node(self, node1, threshold_node,prohibited_node) -> tuple:  # search only in two sides
          """
            return the nearest node for the node1 while note passing threshold_node
            :return: random node
          """
          _node = None
          if node1 is not None and threshold_node is not None:

            row, col = node1
            row1, col1 = threshold_node

            if prohibited_node is None:
                if row == row1:  # in the same line
                    _node1 = (row - 1, col)
                    _node2 = (row + 1, col)

                    if self.within_map_size(_node1):  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.within_map_size(_node2):  # and not self.is_free(_node1):
                        _node = _node2
                    else:
                        _node = None

                elif col == col1:  # the same col
                    _node1 = (row, col - 1)
                    _node2 = (row, col + 1)

                    if self.within_map_size(_node1):  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.within_map_size(_node2):  # and not self.is_free(_node1):
                        _node = _node2
                    else:
                        _node = None
            else:

                if row == row1:  # in the same line
                    _node1 = (row - 1, col)
                    _node2 = (row + 1, col)

                    if self.within_map_size(_node1) and _node1 != prohibited_node:  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.within_map_size(_node2) and _node2 != prohibited_node:  # and not self.is_free(_node1):
                        _node = _node2
                    else:
                        _node = None

                elif col == col1:  # the same col
                    _node1 = (row, col - 1)
                    _node2 = (row, col + 1)

                    if self.within_map_size(_node1) and _node1 != prohibited_node:  # and not self.is_free(_node2):
                        _node = _node1
                    elif self.within_map_size(_node2) and _node2 != prohibited_node:  # and not self.is_free(_node1):
                        _node = _node2
                    else:
                        _node = None

          return _node

    def free_neighboring_node(self, pos, prohibited_nodes):
          """
            find a free node in the neighborhood
          """
          node_ = None
          if pos is not None:
            neighborhood = self.get_neighbors(pos, diagonal=False)

            for node in neighborhood:
                if prohibited_nodes is not None and node in prohibited_nodes:
                    continue
                else:
                    if node in self._graph.nodes and self.is_free(node):
                        node_ = node
                        break
          return node_


    def occupied_neighboring_nodes(self, pos):
            """
            find the occupied neighboring nodes
            """
            neighborhood = self.get_neighbors(pos, diagonal=False)

            ocuupied_nodes = []
            for node in neighborhood:
                if self.within_map_size(node) and node in self._graph.nodes and not self.is_free(node):
                    ocuupied_nodes.append(node)

            return ocuupied_nodes

    def get_nearest_free_node_on_right_left_mode(self, node1, drection_node,mode) -> tuple:
           """
            return the nearest node for the node1 while following the direction_node
            :return: random node
           """

           _node = None
           if drection_node is not None:
            row, col = node1
            row1, col1 = drection_node

            i = 0
            num_tries = 0

            while _node is None or self.is_obstacle(_node) and num_tries < 70:
                num_tries += 1
                i += 1
                if row == row1:  # in the same line
                    if i == 1:
                        if mode ==1:
                          _node = (row1 - 1, col1)  #
                        else:
                          _node = (row1 + 1, col1)
                    if i == 2:
                        if mode == 1:
                          _node = (row1 + 1, col1)
                        else:
                            _node = (row1 - 1, col1)  #

                        if col1 < col:  # search left side
                            col1 -= 1
                        else:  # search right side
                            col1 += 1  # go
                        i = 0


                elif col == col1:  # the same col
                    if i == 1:
                      if mode == 1:
                        _node = (row1, col1 - 1)
                      else:
                         _node = (row1, col1 + 1)
                    if i == 2:
                        if  mode == 1:
                          _node = (row1, col1 + 1)
                        else:
                            _node = (row1, col1 - 1)

                        if row1 < row:  # search Up-side
                            row1 -= 1
                        else:  # search down-side
                            row1 += 1  # go
                        i = 0

            if num_tries == 70:
                _node = None

            return _node

    def get_nearest_free_node_on_right_left(self, node1, drection_node) -> tuple:
            """
            return the nearest node for the node1 while following the direction_node
            :return: random node
            """

            row, col = node1
            row1, col1 = drection_node

            _node = None
            i = 0
            num_tries = 0

            while _node is None or self.is_obstacle(_node) and num_tries < 50:
                num_tries += 1
                i += 1
                if row == row1:  # in the same line
                    if i == 1:
                        _node = (row1 - 1, col1)  #
                    if i == 2:
                        _node = (row1 + 1, col1)

                        if col1 < col:  # search left side
                            col1 -= 1
                        else:  # search right side
                            col1 += 1  # go
                        i = 0


                elif col == col1:  # the same col
                    if i == 1:
                        _node = (row1, col1 - 1)
                    if i == 2:
                        _node = (row1, col1 + 1)

                        if row1 < row:  # search Up-side
                            row1 -= 1
                        else:  # search down-side
                            row1 += 1  # go
                        i = 0

            if num_tries == 50:
                _node = None

            return _node

    def get_nearest_random_free_node(self, mypos):
       _node     = None
       num_tries = 0
       _node = self.free_neighboring_node(mypos, mypos)
       while _node is None or self.is_obstacle(_node) and num_tries < 50:
           num_tries +=1
           neighborhood = self.get_neighbors(mypos, diagonal=False)
           mypos =random.choice(neighborhood)
           _node = self.free_neighboring_node(mypos, mypos)

       return _node


    def move_agent(self, agent: AgentInterface, new_position: Tuple[int, int]):
            assert self._graph.nodes[agent.position]["agent"] == agent, \
                f"Error, agent is not currently located at {agent.position}"
            assert self._graph.nodes[new_position]["agent"] is None, \
                f"Error, location is not free {new_position}"

            self._graph.nodes[agent.position]["agent"] = None
            self._graph.nodes[new_position]["agent"] = agent
            agent.position = new_position

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

    def set_as_path(self, path):
            """
            mark the nodes of a path with the path state
            :return:
            """
            for node in path:
                self._graph.nodes[node]["state"] = 'path'

    def set_as_target(self, node):
            """
            mark the nodes of a path with the path state
            :return:
            """
            self._graph.nodes[node]["state"] = "target"

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