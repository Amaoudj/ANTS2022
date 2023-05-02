""" A smart Agent"""

import logging
import time
import  pyautogui
import numpy as np
import copy
from . agent_interface import AgentInterface

from swarmI4.calculation.path_planning import PathFinder
from swarmI4.map.map import Map

from typing import Callable, Tuple


class SmartAgent(AgentInterface):

    def __init__(self,conf,my_map,position,num_targets):
            super().__init__(conf,position)

            self._path_finder = PathFinder()
            # initialize targets
            self.num_targets = num_targets
            self.target_list = []
            self.targetReached= False

            # create targets automatically
            #for n in range(0, self.num_targets):
            #   target = my_map.get_random_free_node()

            # initial path info
            self.path = None
            self.remaining_path = None
            self._current_target_id = 0

            self._current_waypoint_id = 0
            self.moving_away = False
            self.next_target = None
            self.next_waypoint = None
            self.critic_node = None
            self.waiting_steps = 0
            self.my_move_away_node = None
            self.got_priority_last_step = False
            self.last_node = None
            self.previous_foundPath = None
            self.my_pos_will_be_free = False
            self.moving_backward = False

            self.my_predecessors = []
            self.leader = None
            self.special_case = False
            self.follower = None
            self.followers = []

            # list of neighbors
            self.neighbors = []
            self.repeated_nodes = []
            self.all_repeated_nodes = []
            self.priority_neighbor = None
            self.action = "wait"
            self.got_conflict = False
            self.conflict_agent = None
            self.got_opposite_conflict = False
            self.started = False
            self.target = None
            self.num_TRIES = 0
            # DATA TO BE STORED
            self.steps = 0
            self.changed_action = False
            self.im_done = False
            self.is_last_node = False
            self.num_conflicts = 0
            self.Graph_copy = None
            self.special_path = []
            self.num_pos_requests = 0
            self.num_followers = 0
            self.goal_changed = False
            self.delay = False
            self.has_new_target = False
            self.has_delayed = False
            self.delay_steps = 0
            self.num_replanned_paths = 0
            self.waitingThreshold = 8
            self.nodesThreshold = 7
            self.repetitionThreshold = 3

            # ***************** rules orders****************************
            self.priorityRuleTreeIntersectionConf = []
            self.priorityRuleTreeOppositeConf = []

            self.rules_order_intersection_conflict = []
            self.rules_order_opposite_conflict = []
            self.rules_order_Pos_Coordination = []

            self.storage_container = [{'sim_time': 0, 'steps': self.steps, 'num_targets': len(self.target_list),
                                       'num_conflicts': self.num_conflicts}]

    def send_my_data(self, map: Map):
            """
            send the agent data to its neighbors
            """

            data = {"AgentID": self.id,
                    "im_done": self.im_done,
                    "pos": self.position,
                    "remaining_nodes": len(self.remaining_path) if not self.im_done else 0,
                    "next_node": self.remaining_path[0] if len(self.remaining_path) > 0 else self.target_list[-1],
                    "next_next_node": self.remaining_path[1] if len(self.remaining_path) > 1 else self.target_list[-1],
                    "num_pos_requests": self.num_pos_requests,
                    "successors": self.num_followers,
                    "got_conflict": self.got_conflict,
                    "changed_action": self.changed_action,
                    "moving_away": self.moving_away,
                    "moving_backward": self.moving_backward,
                    "conflict_agent": self.conflict_agent,
                    "my_last_node": self.last_node,
                    "got_priority_last_step": self.got_priority_last_step,
                    "planned_action": self.action,
                    "target": self.target
                    }

            # map.msg_box[self.id] = data

            map.neighbors_agents_stat[self.id] = data

    def send_my_position(self, map: Map):
            """
            send the agent data to its neighbors
            """
            num_pos_requests, num_followers = 0, 0
            data = {"AgentID": self.id,
                    "im_done": self.im_done,
                    "pos": self.position,
                    "remaining_nodes": 0,
                    "next_node": None,
                    "next_next_node": None,
                    "num_pos_requests": 0,
                    "successors": 0,
                    "got_conflict": 0,
                    "changed_action": self.changed_action,
                    "moving_away": self.moving_away,
                    "moving_backward": self.moving_backward,
                    "conflict_agent": self.conflict_agent,
                    "my_last_node": self.last_node,
                    "got_priority_last_step": self.got_priority_last_step,
                    "planned_action": self.action,
                    "target": self.target
                    }

            map.neighbors_agents_stat[self.id] = data


    def readParameterConfiguration(self):
        self.waitingThreshold = 5



    def is_agent_involved_in_opposite_conflict(self, map, agent):

            return_ = False
            for msg in map.neighbors_agents_stat:
                if msg["AgentID"] == agent["AgentID"]:
                    continue
                else:
                    if (msg["next_node"] == agent["pos"]) and (agent["next_node"] == msg["pos"]):
                        return_ = True
                        break

            return return_

    def is_agent_involved_in_intersection_conflict(self, map, agent):

            return_ = False
            for msg in map.neighbors_agents_stat:  # map.msg_box.values():
                if msg["AgentID"] == agent["AgentID"]:
                    continue
                elif (msg["next_node"] == agent["next_node"]):
                    return_ = True
                    break

            return return_

    def get_follower_agent(self, map, agent_id):
            """
             return the agent having my pos as it next-node
            """
            # concerned_Agent=None
            follower = None
            # if agent_id is not None:
            # for agent in map.neighbors_agents_stat:#map.msg_box.values():
            #   if agent['AgentID'] == agent_id:
            #        concerned_Agent = agent
            #        break
            concerned_Agent = map.neighbors_agents_stat[agent_id]

            for agent in map.neighbors_agents_stat:  # map.msg_box.values():
                if agent is not None and concerned_Agent is not None and agent['AgentID'] != agent_id:
                    if (concerned_Agent['pos'] == agent['next_node']) and (concerned_Agent['next_node'] != agent['pos']):
                        follower = agent
                        break

            return follower

    def get_followers(self, map):

            _followers = []
            agent_id = None
            follower = self.get_follower_agent(map, self.id)  # get first follower
            if follower is not None:
                _followers.append(follower)
                agent_id = follower['AgentID']
            steps = 0

            while agent_id is not None and agent_id != self.id and steps < 20:
                follower = self.get_follower_agent(map, agent_id)
                if follower is None or follower['AgentID'] in (_f['AgentID'] for _f in _followers):
                    break
                _followers.append(follower)
                agent_id = follower['AgentID']
                steps += 1

            return _followers

    def num_pos_requests_and_followers(self, map, position: tuple) -> tuple:
            """
            Find the agents requesting this position in the next or the next-next step and successors
            args: position:(tuple) the coordinates of the node ( row,col)
            return:   num_pos_requests_and_followers
            """
            pos_requests = 0
            num_successors = len(self.followers)  # self.get_followers(map)

            # successors + other agents having 'next_next_node' = my_pos
            for agent in map.neighbors_agents_stat:  # map.msg_box.values():
                if agent['AgentID'] != self.id and ((agent['next_node'] == position) or (agent['next_next_node'] == position)):
                    pos_requests += 1

            return pos_requests, num_successors

    def get_back_node(self, pos, threch):
            x, y = pos
            x1, y1 = threch
            direct_node = None
            if x == x1:
                if y1 > y:
                    direct_node = (x, y - 1)
                else:
                    direct_node = (x, y + 1)

            elif y == y1:
                if x1 > x:
                    direct_node = (x - 1, y)
                else:
                    direct_node = (x + 1, y)
            return direct_node

    def get_leader_agent(self, map, agent_id):
            """
              return the leader agent that is in my next-node
            """
            predecessor = None
            concerned_Agent = map.neighbors_agents_stat[agent_id]

            for agent in map.neighbors_agents_stat:  # map.msg_box.values():
                if (agent is not None and concerned_Agent is not None) and (agent['AgentID'] != agent_id):
                    if (concerned_Agent['next_node'] == agent['pos']) and (
                            agent['next_node'] != concerned_Agent['pos']) and (
                            agent['next_next_node'] != concerned_Agent['pos']):
                        predecessor = agent
                        break

            return predecessor

    def get_predecessors(self, map):
            """
            return the list of agent in front of me
            """
            _predecessors = []
            predecessor = self.get_leader_agent(map, self.id)
            if predecessor is not None and predecessor['AgentID'] != self.id:
                _predecessors.append(predecessor)
                agent_id = predecessor['AgentID']
            steps = 0

            while (steps < 30 and predecessor is not None and predecessor['AgentID'] != self.id and predecessor['AgentID'] != _predecessors[-1]['AgentID']):  #
                predecessor = self.get_leader_agent(map, agent_id)
                steps += 1
                if predecessor is not None and predecessor not in _predecessors and predecessor['AgentID'] != \
                        _predecessors[-1]['AgentID']:
                    _predecessors.append(predecessor)
                    agent_id = predecessor['AgentID']
                else:
                    break

            return _predecessors

    # this function will update the neighbors list of this agent
    def get_neighbors(self, map):
            neighbors = []
            my_next_node = None
            if len(self.remaining_path) > 0:
                my_next_node = self.remaining_path[0]

            for msg in map.neighbors_agents_stat:  # map.msg_box.values():
                if msg["next_node"] != None:
                    if (msg["next_node"] == my_next_node) or \
                            (msg["next_node"] == self._position) or \
                            (msg["pos"] == my_next_node) or \
                            msg["AgentID"] == self.id:
                        neighbors.append(msg)
            return neighbors

    def get_intersection_conflict_neighbors(self, map):
            """
            return: the neighbors list of this agent
            """
            neighbors = []
            my_next_node = None
            if len(self.remaining_path) > 0:
                my_next_node = self.remaining_path[0]

            for msg in map.neighbors_agents_stat:  # map.msg_box.values():
                if msg["next_node"] != None:  # and not self.is_agent_implied_in_opposite_conflict(map,msg):
                    if (msg["next_node"] == my_next_node) or msg["AgentID"] == self.id:
                        neighbors.append(msg)

            return neighbors

    def get_opposite_conflict_neighbors(self, map):
            """
            return: the neighbors list of this agent
            """
            neighbors = []
            my_next_node = None
            if len(self.remaining_path) > 0:
                my_next_node = self.remaining_path[0]

            for msg in map.neighbors_agents_stat:  # map.msg_box.values():
                if msg["next_node"] != None:
                    if (msg["next_node"] == self._position and msg["pos"] == my_next_node) or \
                            msg["AgentID"] == self.id:
                        neighbors.append(msg)
            return neighbors

    def is_free(self, node, neighbors):
            ret = True
            for agent in neighbors:
                if agent['pos'] == node:
                    ret = False
                    break

            return ret

    def is_target_between_two_nodes(self, node, node1, node2):
            ret = False
            if node is not None and node1 is not None and node2 is not None:
                x, y = node
                x1, y1 = node1
                x2, y2 = node2

                if (x1 == x2 + 1) or (x1 == x2 - 1) or (x2 == x1 + 1) or (x2 == x1 - 1):
                    if ((y1 > y2) and (y < y1 and y > y2)) or ((y1 < y2) and (y < y2 and y > y1)):
                        ret = True

                elif (y1 == y2 + 1) or (y1 == y2 - 1) or (y2 == y1 + 1) or (y2 == y1 - 1):
                    if ((x1 > x2) and (x < x1 and x > x2)) or ((x1 < x2) and (x < x2 and x > x1)):
                        ret = True

            return ret

    def is_node_between_two_nodes(self, node, node1, node2):
            ret = False
            if node is not None and node1 is not None and node2 is not None:
                x, y = node
                x1, y1 = node1
                x2, y2 = node2

                if (x1 == x2) and ((y < y1 and y > y2) or (y < y2 and y > y1)):
                    ret = True
                elif (y1 == y2) and ((x < x1 and x > x2) or (x < x2 and x > x1)):
                    ret = True

            return ret

    def get_agent_object(self, agent_id):
         return map.neighbors_agents_stat[agent_id]

    def check_for_conflict(self, map):
            # if len(self.remaining_path) >= 1:
            my_next_node = self.remaining_path[0]  # self.next_waypoint
            # else:
            # my_next_node = self.target_list[-1]
            next_nodes = []
            is_opposite_conflict = False
            for msg in map.neighbors_agents_stat:  # map.msg_box.values():
                if msg["AgentID"] == self.id:
                    continue
                else:
                    # 1-opposite conflict (two conflict nodes)
                    if (msg["next_node"] == self.position) and (my_next_node == msg["pos"]):
                        self.neighbors = self.get_opposite_conflict_neighbors(map)
                        next_nodes.append(msg["next_node"])
                        next_nodes.append(my_next_node)
                        is_opposite_conflict = True
                        return True, next_nodes

                    # -intersection conflict
                    elif (msg["next_node"] == my_next_node):
                        if not is_opposite_conflict:
                            self.neighbors = self.get_intersection_conflict_neighbors(map)
                            # remove the agents having opposite_conflict then check if len (self.neighbors)>1
                            for agent in self.neighbors:
                                if self.is_agent_involved_in_opposite_conflict(map, agent):
                                    self.neighbors.remove(agent)
                            if len(self.neighbors) > 1:
                                next_nodes.append(my_next_node)
                                return True, next_nodes
                            else:
                                continue

            self.neighbors = []
            return False, None

    def compute_local_data(self, map):

            self.my_predecessors.clear()
            self.leader = self.get_leader_agent(map, self.id)
            self.my_predecessors = self.get_predecessors(map)
            self.follower = self.get_follower_agent(map, self.id)
            self.followers = self.get_followers(map)
            self.num_pos_requests, self.num_followers = self.num_pos_requests_and_followers(map, self.position)
            self.send_my_data(map)

    def handle_conflicts(self, map):

            if len(self.remaining_path) >= 1:
                my_next_node = self.next_waypoint
            else:
                my_next_node = self.target_list[-1]

            # solve conflicts for all agents
            is_conflict, critic_node = self.check_for_conflict(map)

            if is_conflict:
                # print("is_conflict is True ...")

                self.got_conflict = True

                if len(critic_node) == 1:
                    _node = critic_node[0]
                    if type(_node) is list:
                        self.critic_node = _node[0]
                    else:
                        self.critic_node = _node

                    # if not map.occupied(self.critic_node):
                    self.solve_intersection_conflict(map, self.critic_node)
                else:
                    # print("solve_opposite_conflict ....")
                    self.got_opposite_conflict = True
                    self.solve_opposite_conflict(map, critic_node)
                    # print("passed solve_opposite_conflict ....")
            else:  # check if there is successor agent has the longest path: if yes, move away and let him pass
                self.my_pos_will_be_free = True
                self.action = "move"
                self.got_conflict = False

                successor = self.follower  # self.get_follower_agent(map, self.id)
                if successor is not None and successor["AgentID"] != self.id:

                    if not self.is_agent_involved_in_opposite_conflict(map, successor) and not self.is_agent_involved_in_intersection_conflict(map, successor):
                        if (successor["next_node"] == self.position) and (successor["next_next_node"] == my_next_node) and successor["remaining_nodes"] > len(self.remaining_path) + 1 :
                            #if (len(self.remaining_path) > 10 and (successor["remaining_nodes"] > len(self.remaining_path) + 1)) or (len(self.remaining_path) < 6 and (successor["remaining_nodes"] > len(self.remaining_path))):

                                got_free_node = map.get_right_or_left_free_node(self.position, successor['pos'], successor['next_next_node'])

                                for msg in map.neighbors_agents_stat:  # map.msg_box.values():
                                    if len(self.remaining_path) > 10 and (msg["next_node"] == got_free_node or msg[
                                        "next_next_node"] == got_free_node):  # check if this node is the node of another agent
                                        got_free_node = None
                                        break

                                if got_free_node is not None:
                                    self.moving_away = True
                                    self.action = "Let_agent_pass"
                                    # self.agent_I_gave_him_way=successor["AgentID"]
                                    self.remaining_path[0:0] = [got_free_node, self.position]  #
                                    # logging.info(f'Agent{self.id} in {self.position} gives way to let agent{successor["AgentID"]} pass {self.remaining_path}')
                                    self.send_my_data(map)

    def solve_intersection_conflict(self, map, critic_node: tuple) -> tuple:
        """
         return the action of an agent for the next step
        """
        solution = {}
        is_critic_node_free = True
        id_critic = None
        self.critic_node = critic_node
        candidates = self.neighbors.copy()

        if self.neighbors[0]['AgentID'] == self.id:
            self.conflict_agent = self.neighbors[1]['AgentID']
        else:
            self.conflict_agent = self.neighbors[0]['AgentID']

        priority_agent = None

        # rule0>> got priority in the previous time step
        for agent in candidates:
            if (agent["got_priority_last_step"] and int(
                    agent['remaining_nodes']) > 1):  # or agent['moving_away']: #or agent["moving_away"]:
                priority_agent = agent['AgentID']
                if priority_agent == self.id:  # it is me
                    self.got_priority_last_step = False  #
                break

        for agent in candidates:
            if int(agent['remaining_nodes']) == 1:
                candidates.remove(agent)
                break

        if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']

        if priority_agent is None and len(candidates) > 1:

            if priority_agent is None and len(candidates) == 2:

                got_free_node1 = map.get_right_or_left_free_node(candidates[0]["pos"], candidates[1]["pos"], candidates[1]["next_next_node"])  #############
                got_free_node2 = map.get_right_or_left_free_node(candidates[1]["pos"], candidates[0]["pos"],candidates[0]["next_next_node"])  #############

                if got_free_node1 is None and got_free_node2 is not None:  # give the priority to this agent (it hasn't any free neighboring_node to got to)
                    priority_agent = candidates[0]['AgentID']

                elif got_free_node2 is None and got_free_node1 is not None:
                    priority_agent = candidates[1]['AgentID']

            if priority_agent is None :
              if len(candidates) == 2:
                for agent in candidates:
                    if agent['moving_backward'] or agent['moving_away']:
                        for n in candidates:
                            if n != agent:
                                if agent['conflict_agent'] == n['AgentID']:
                                    priority_agent = n['AgentID']  #  robot Agent is moving out of the robot n, so  robot n is given priority to avoid livelock
              elif  len(candidates) >  2: # give priority to the robot moving out of the way
                new_candidates= []
                for agent in candidates:
                  if agent['moving_backward'] or agent['moving_away']:
                    new_candidates.append(agent)
                if len(new_candidates) == 1:  # if only one candidate left then it will have the priority
                   priority_agent = new_candidates[0]['AgentID']

            if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02
                # Find the maximum remaining nodes among all agents
                max_remaining = max(agent['remaining_nodes'] for agent in candidates)
                # Count the number of agents with the maximum remaining nodes
                count = sum(1 for agent in candidates if agent['remaining_nodes'] == max_remaining)

                if count == 1:  # only one agent
                    remaining_node = [agent['remaining_nodes'] for agent in candidates]
                    indx = remaining_node.index(max_remaining)
                    priority_agent = candidates[indx]['AgentID']


            if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

                highest_requests_num = []
                done = False
                for i in candidates:
                    highest_requests_num.append(i['successors'])
                    # if i["im_done"]:
                    # done=True
                    # break
                if not done:  # if an agent is done, do not applied this rule
                    indx = highest_requests_num.index(max(highest_requests_num))
                    highest_requests = highest_requests_num[indx]
                    newcandidates = [i for i in candidates if i['successors'] == highest_requests]
                    candidates.clear()
                    candidates.extend(newcandidates)

                if len(candidates) == 1:  # if only one candidate left then it will have the priority
                    priority_agent = candidates[0]['AgentID']

                    # if self.neighbors[0]['AgentID'] == self.id:
                    #   logging.info(f'priority rule 02: True for agent{priority_agent}')

            if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

                highest_requests_num = []
                for i in candidates:
                    highest_requests_num.append(i['num_pos_requests'])

                indx = highest_requests_num.index(max(highest_requests_num))
                highest_requests = highest_requests_num[indx]
                newcandidates = [i for i in candidates if i['num_pos_requests'] == highest_requests]
                candidates.clear()
                candidates.extend(newcandidates)

                if len(candidates) == 1:  # if only one candidate left then it will have the priority
                    priority_agent = candidates[0]['AgentID']


                # rule 05: in case every other rule failed , choose the agent with the highest id
                else:
                    priority_agent = max([agent['AgentID'] for agent in candidates])
                    # if self.neighbors[0]['AgentID'] == self.id:
                    # logging.info( f'last rule failed--we take the agent with the highest id : (agent{priority_agent})')

        prohibited_node = None
        threshold_node = None
        # update variables:
        self.moving_away = False
        self.moving_backward = False
        self.got_priority_last_step = False  #
        self.action = None

        agent_having_priority = None
        for agent in self.neighbors:
            if agent['AgentID'] == priority_agent:
                agent_having_priority = agent
                break

        if len(self.neighbors) == 5:  # the critic node occupied by an agent

            agent_to_give_way = None
            free_node_to_give_way = None
            agent_done = None
            moveAGVnode = None
            solved = False
            got_to_node = None

            if self.neighbors[0]['im_done'] or self.neighbors[1]['im_done'] or self.neighbors[2]['im_done'] or \
                    self.neighbors[3]['im_done'] or self.neighbors[4]['im_done']:

                for agent in self.neighbors:
                    if agent['im_done']:  # self.im_done :#and self.action=="wait" : # :

                        agent_done = agent['AgentID']
                        solution[agent['AgentID']] = "move"
                        got_to_node = map.free_neighboring_node(agent['pos'], self.neighbors)
                        if got_to_node is not None:
                            solved = True

                        else:  # choose the other agent node
                            solved = False
                            solution[agent['AgentID']] = "move"  # _to_AGV_node
                            # chose the node of the agent to move to
                            condidate_ = self.neighbors.copy()
                            condidate_.remove(agent)  # remove my self from the neighbors
                            if agent_having_priority in condidate_:
                                condidate_.remove(
                                    agent_having_priority)  # remove the agent having the priority from the list
                            moveAGVnode, agent_to_give_way = self.get_best_AGV_node(map, condidate_, critic_node)
                            # the action of the agent located in this node
                            free_node_to_give_way, _ = map.get_WayNode_include_moveBackward(agent_to_give_way['pos'],
                                                                                            critic_node)

                if solved:
                    for agent in self.neighbors:

                        if agent['AgentID'] == agent_done:
                            if agent_done == self.id:
                                self.im_done = False
                                self.is_last_node = False

                                solution[agent['AgentID']] = "move"
                                self.remaining_path[0:0] = [got_to_node]

                        else:
                            solution[agent['AgentID']] = "wait"
                else:

                    for agent in self.neighbors:

                        if agent['AgentID'] == agent_done:
                            solution[agent['AgentID']] = "move"
                            if agent_done == self.id:
                                self.im_done = False
                                self.is_last_node = False
                                self.remaining_path[0:0] = [moveAGVnode]  # , self.critic_node

                                self.moving_away = True

                        elif agent['AgentID'] == agent_to_give_way['AgentID']:
                            solution[agent_to_give_way['AgentID']] = "move_to_node_and_wait"
                            if agent_to_give_way['AgentID'] == self.id:  # it is me
                                self.my_move_away_node = free_node_to_give_way
                        else:
                            solution[agent['AgentID']] = "wait"


            else:
                for n in self.neighbors:
                    if n['pos'] == critic_node:
                        solution[n['AgentID']] = "move"
                        if n['AgentID'] == self.id:
                            self.moving_away = True
                    else:
                        solution[n['AgentID']] = "wait"

        if len(self.neighbors) == 4:  # 4 AGVs in an intersection
            moveAGVnode = None

            if self.is_free(critic_node, self.neighbors):
                solution[agent_having_priority['AgentID']] = "move"

                for n in self.neighbors:
                    if n['pos'] == agent_having_priority[
                        'next_next_node']:  # and n['AgentID']!= agent_having_priority['AgentID']

                        # get a free node without moving back
                        got_free_node1 = map.get_right_or_left_free_node(n['pos'], critic_node, critic_node)

                        if got_free_node1 is None:
                            solution[n['AgentID']] = "move_to_AGV_node"
                            solution[agent_having_priority['AgentID']] = "wait"

                            # chose the node of the agent to move to
                            condidate_ = self.neighbors.copy()
                            condidate_.remove(n)  # remove my self from the neighbors
                            if agent_having_priority in condidate_:
                                condidate_.remove(
                                    agent_having_priority)  # remove the agent having the priority from the list
                            moveAGVnode, agent_ = self.get_best_AGV_node(map, condidate_, critic_node)

                            #############################################################################################################
                            # the action of the agent located in the node
                            got_free_node3, _ = map.get_WayNode_include_moveBackward(agent_['pos'], critic_node)
                            solution[agent_['AgentID']] = "move_to_node_and_wait"
                            if agent_['AgentID'] == self.id:  # it is me
                                self.my_move_away_node = got_free_node3
                            #############################################################################################################

                            if n['AgentID'] == self.id:  # to update the AGV node o move to
                                self.my_move_away_node = moveAGVnode
                            if agent_having_priority['AgentID'] == self.id:
                                self.got_priority_last_step = True


                        else:  # if (got_free_node1 != msg['next_node'] for msg in map.msg_box.values() ):
                            solution[agent_having_priority['AgentID']] = "move"
                            solution[n['AgentID']] = "move_to_node_and_wait"
                            if n['AgentID'] == self.id:  # it is me
                                self.my_move_away_node = got_free_node1

                    else:
                        if n['AgentID'] != agent_having_priority['AgentID']:
                            solution[n['AgentID']] = "wait"
                            if n['AgentID'] == self.id:
                                self.action == "wait"

            else:

                if self.neighbors[0]['im_done'] or self.neighbors[1]['im_done'] or self.neighbors[2]['im_done'] or \
                        self.neighbors[3]['im_done']:

                    agent_to_give_way = None
                    free_node_to_give_way = None
                    agent_done = None
                    moveAGVnode = None
                    solved = False
                    got_to_node = None

                    for agent in self.neighbors:
                        if agent['im_done']:  # self.im_done :#and self.action=="wait" : # :

                            solution[agent['AgentID']] = "move"
                            agent_done = agent['AgentID']
                            got_to_node = map.free_neighboring_node(agent['pos'], self.neighbors)

                            if got_to_node is not None:
                                solved = True

                            else:  # choose the other agent node
                                solved = False

                                # chose the node of the agent to move to
                                condidate_ = self.neighbors.copy()
                                condidate_.remove(agent)  # remove my self from the neighbors
                                if agent_having_priority in condidate_:
                                    condidate_.remove(
                                        agent_having_priority)  # remove the agent having the priority from the list

                                moveAGVnode, agent_to_give_way = self.get_best_AGV_node(map, condidate_, critic_node)
                                # the action of the agent located in this node
                                free_node_to_give_way, _ = map.get_WayNode_include_moveBackward(
                                    agent_to_give_way['pos'], critic_node)

                    if solved:
                        for agent in self.neighbors:

                            if agent['AgentID'] == agent_done:
                                if agent_done == self.id:
                                    self.im_done = False
                                    self.is_last_node = False

                                    solution[agent['AgentID']] = "move"
                                    self.remaining_path[0:0] = [got_to_node]

                            else:
                                solution[agent['AgentID']] = "wait"
                    else:

                        for agent in self.neighbors:

                            if agent['AgentID'] == agent_done:
                                solution[agent['AgentID']] = "move"
                                if agent_done == self.id:
                                    self.im_done = False
                                    self.is_last_node = False
                                    self.remaining_path[0:0] = [moveAGVnode]  # , self.critic_node

                                    self.moving_away = True

                            elif agent['AgentID'] == agent_to_give_way['AgentID']:
                                solution[agent_to_give_way['AgentID']] = "move_to_node_and_wait"
                                if agent_to_give_way['AgentID'] == self.id:  # it is me
                                    self.my_move_away_node = free_node_to_give_way
                            else:
                                solution[agent['AgentID']] = "wait"

                else:
                    for n in self.neighbors:
                        if n['pos'] == critic_node:
                            solution[n['AgentID']] = "move"
                            if n['AgentID'] == self.id:
                                self.moving_away = True
                        else:
                            solution[n['AgentID']] = "wait"

        elif len(self.neighbors) == 3:
            is_free = True
            for n in self.neighbors:
                if n['AgentID'] != agent_having_priority['AgentID']:
                    if agent_having_priority['next_next_node'] == n['pos'] or not self.is_free(critic_node,
                                                                                               self.neighbors):  # means that the agent having priority will move to an occupied node
                        is_free = False  # the next_next_node is not free

                if n['pos'] == critic_node:
                    is_critic_node_free = False
                    id_critic = n['AgentID']
                    prohibited_node = agent_having_priority['pos']

            if is_free:
                solution[agent_having_priority['AgentID']] = "move"
                for n in self.neighbors:
                    if agent_having_priority['AgentID'] != n['AgentID']:
                        solution[n['AgentID']] = "wait"

            elif self.is_free(critic_node,
                              self.neighbors):  # not Free, so the agent in this node should go to a free node and let the priority to the agent
                solution[agent_having_priority['AgentID']] = "move"
                for n in self.neighbors:
                    if agent_having_priority['next_next_node'] == n['pos']:  # the agent accupied this node
                        got_free_node = map.get_right_or_left_free_node(n['pos'], critic_node, critic_node)
                        # got_free_node1 = map.get_right_or_left_free_node(agent_having_priority['pos'], critic_node,critic_node)
                        if got_free_node is not None:
                            solution[n['AgentID']] = "move_to_node_and_wait"
                            solution[agent_having_priority['AgentID']] = "move"  # "wait"

                            if n['AgentID'] == self.id:  # it is me
                                self.my_move_away_node = got_free_node

                        else:

                            # if map.is_free(critic_node):
                            got_free_node1 = map.free_neighboring_node(critic_node, self.neighbors)
                            if got_free_node1 is not None:
                                solution[n['AgentID']] = "move_to_node_via_critic_node"
                                if n['AgentID'] == self.id:  # it is me
                                    self.my_move_away_node = got_free_node1
                                solution[agent_having_priority['AgentID']] = "wait"
                                if agent_having_priority['AgentID'] == self.id:  # it is me
                                    self.got_priority_last_step = True

                            else:
                                solution[agent_having_priority['AgentID']] = "wait"
                                solution[n['AgentID']] = "move"  # move only to critic node
                        # else:


                    else:
                        if n['AgentID'] != agent_having_priority['AgentID']:
                            solution[n['AgentID']] = "wait"

            else:  # the agent in the critic node should move out away

                agent_to_give_way = None
                free_node_to_give_way = None
                agent_done = None
                moveAGVnode = None
                solved = False
                got_to_node = None

                if self.neighbors[0]['im_done'] or self.neighbors[1]['im_done'] or self.neighbors[2]['im_done']:

                    for agent in self.neighbors:
                        if agent['im_done']:  # self.im_done :#and self.action=="wait" : # :

                            agent_done = agent['AgentID']
                            solution[agent['AgentID']] = "move"
                            got_to_node = map.free_neighboring_node(agent['pos'], self.neighbors)
                            if got_to_node is not None:
                                solved = True

                            else:  # choose the other agent node
                                solution[agent['AgentID']] = "move"  # _to_AGV_node
                                # chose the node of the agent to move to
                                condidate_ = self.neighbors.copy()
                                condidate_.remove(agent)  # remove my self from the neighbors
                                if agent_having_priority in condidate_:
                                    condidate_.remove(
                                        agent_having_priority)  # remove the agent having the priority from the list
                                moveAGVnode, agent_to_give_way = self.get_best_AGV_node(map, condidate_, critic_node)
                                # the action of the agent located in this node
                                free_node_to_give_way, _ = map.get_WayNode_include_moveBackward(
                                    agent_to_give_way['pos'], critic_node)

                    if solved:
                        for agent in self.neighbors:

                            if agent['AgentID'] == agent_done:
                                if agent_done == self.id:
                                    self.im_done = False
                                    self.is_last_node = False

                                    solution[agent['AgentID']] = "move"
                                    self.remaining_path[0:0] = [got_to_node]
                                    # logging.info(f'agent{self.id} done --> Solved--remaining path: {self.remaining_path}')
                            else:
                                solution[agent['AgentID']] = "wait"

                    else:

                        for agent in self.neighbors:

                            if agent['AgentID'] == agent_done:
                                solution[agent['AgentID']] = "move"
                                if agent_done == self.id:
                                    self.im_done = False
                                    self.is_last_node = False
                                    self.remaining_path[0:0] = [
                                        moveAGVnode]  # , self.critic_node   ################################################
                                    # logging.info(f'Agent{self.id} move_to_AGV_node--remaining path: {self.remaining_path}')
                                    self.moving_away = True

                            elif agent['AgentID'] == agent_to_give_way['AgentID']:
                                solution[agent_to_give_way['AgentID']] = "move_to_node_and_wait"
                                if agent_to_give_way['AgentID'] == self.id:  # it is me
                                    self.my_move_away_node = free_node_to_give_way
                            else:
                                solution[agent['AgentID']] = "wait"

                else:
                    for n in self.neighbors:
                        if n['pos'] == critic_node:
                            solution[n['AgentID']] = "move"
                            if n['AgentID'] == self.id:
                                self.moving_away = True  # next step you need to mve
                        else:
                            solution[n['AgentID']] = "wait"

        elif len(self.neighbors) == 2:  # if there are only 2 AGVs

            if self.is_free(critic_node, self.neighbors):  # no agent in the critic_node
                for n in self.neighbors:
                    if n['AgentID'] != agent_having_priority['AgentID']:
                        if n['pos'] == agent_having_priority[
                            'next_next_node']:  # means the other agent located on my node

                            got_free_node = map.get_right_or_left_free_node(n['pos'], critic_node, critic_node)
                            got_free_node1 = map.get_right_or_left_free_node(agent_having_priority['pos'], critic_node,
                                                                             critic_node)

                            if got_free_node is not None and got_free_node != n[
                                'pos']:  # and got_free_node != critic_node:
                                solution[agent_having_priority['AgentID']] = "move"
                                solution[n['AgentID']] = "move_to_node_and_wait"

                                if n['AgentID'] == self.id:  # it's me
                                    self.my_move_away_node = got_free_node

                            # change the priority if the other agent has a neighbour free node
                            elif got_free_node1 is not None and int(n['remaining_nodes']) > 1 and got_free_node1 != \
                                    agent_having_priority['pos'] and got_free_node1 != critic_node:
                                solution[agent_having_priority['AgentID']] = "move_to_node_and_wait"

                                if agent_having_priority['AgentID'] == self.id:  # it is me
                                    self.my_move_away_node = got_free_node1

                                solution[n['AgentID']] = "move"
                                priority_agent = n['AgentID']

                            else:
                                got_free_node2 = map.free_neighboring_node(critic_node, self.neighbors)
                                # got_free_node2, _ = map.get_right_or_left_free_node(critic_node, agent_having_priority['pos'],n['pos'])
                                if got_free_node2 is not None:
                                    solution[n['AgentID']] = "move_to_node_via_critic_node"
                                    solution[agent_having_priority['AgentID']] = "wait"
                                    if n['AgentID'] == self.id:  # it is me
                                        self.my_move_away_node = got_free_node2

                                else:  # if None
                                    # got_free_node2,_ = map.get_WayNode_include_moveBackward(n['pos'], critic_node)
                                    solution[agent_having_priority['AgentID']] = "move"
                                    threshold_node = critic_node
                                    prohibited_node = agent_having_priority['next_next_node']
                                    # muveBackward
                                    solution[n['AgentID']] = "move_out_of_the_way_intersection_"
                                    if n['AgentID'] == self.id:
                                        self.move_out_of_the_way_intersection(map, self.critic_node, prohibited_node)
                                        self.moving_backward = True



                        else:  # this means that the next_next_node of agent_having_priority is free
                            solution[agent_having_priority['AgentID']] = "move"
                            solution[n['AgentID']] = "wait"

            else:  # the agent in the critic node should move out away

                agent_done = None
                agent_moving = None

                special_traitement = False
                if self.neighbors[0]['im_done'] or self.neighbors[1]['im_done']:

                    for agent_ in self.neighbors:
                        if not agent_['im_done']:
                            agent_moving = agent_
                        elif agent_['im_done']:
                            agent_done = agent_

                    #print(agent_done,agent_moving)

                    if self.neighbors[0]['im_done'] or self.neighbors[1]['im_done']:

                        # print (agent_done['pos'])
                        go_to_node1, _ = map.get_Free_WayNode(agent_done['pos'], agent_moving['pos'], agent_moving['next_next_node'])  # get_Free_WayNode
                        # if go_to_node is None :
                        # go_to_node,_= map.get_Free_WayNode(agent_done['pos'], agent_moving['pos'], agent_moving['pos'])
                        if go_to_node1 is not None:
                            solution[agent_done['AgentID']] = "move_right_left_backward"
                            solution[agent_moving['AgentID']] = "move"

                            if agent_done['AgentID'] == self.id:
                                self.im_done = False
                                self.is_last_node = False
                                self.action == "move_right_left_backward"
                                solution[agent_done['AgentID']] = "move_right_left_backward"

                                self.moving_away = True
                                self.moving_backward = True

                                self.remaining_path[0:0] = [go_to_node1]  # , self.position
                                self.special_case = True
                                self.repeated_nodes.clear()
                                self.special_path = [go_to_node1]  # , self.position

                            if agent_moving['AgentID'] == self.id:
                                #logging.info(f'agent{self.id} can move then')
                                self.action == "move"
                                solution[agent_moving['AgentID']] = "move"

                                self.special_case = False
                                # self.repeated_nodes.clear()
                                # self.special_path.clear()
                                # node1 = map.get_nearest_free_node_on_right_left(self.position, agent_done['pos'],2)
                                # path = self._path_finder.astar_planner(map._copy_graph, self.position, node1)
                                # if path is not None and len(path) > 0:
                                # self.special_path = path

                        else:  # no right/left free node
                            # 1-check the number of free neighboring node of the other agent
                            neighbors = map.get_neighbors(agent_moving['pos'], diagonal=False)
                            for neighbor in neighbors:
                                if self.is_node_between_two_nodes(neighbor, agent_moving['pos'],
                                                                  agent_done['pos']) or neighbor == agent_done['pos']:
                                    neighbors.remove(neighbor)

                            number_free_neighbors = map.get_number_of_free_neighbors(neighbors)

                            if number_free_neighbors > 1:
                                # this agent will move to a free node and the agent_done will move to the other free node via this node
                                free_node1 = map.free_neighboring_node(agent_moving['pos'], self.neighbors)
                                free_node2 = map.free_neighboring_node(agent_moving['pos'], [free_node1])
                                solution[agent_done['AgentID']] = "move_to_robot_neighbor"
                                solution[agent_moving['AgentID']] = "move_to_free_neighbor_node"

                                self.special_case = True
                                self.repeated_nodes.clear()
                                # self.special_path.clear()

                                if agent_done['AgentID'] == self.id:
                                    self.im_done = False
                                    self.action == "move_to_robot_nieghbor"

                                    self.is_last_node = False
                                    self.moving_away = True

                                    self.remaining_path[0:0] = [agent_moving['pos'], free_node2, agent_moving['pos']]  #
                                    # logging.info(f'agent{self.id} done --> move to neighbor of the robot {self.remaining_path}')

                                    self.special_path[0:0] = [agent_moving['pos'], free_node2, agent_moving['pos']]  #

                                if agent_moving['AgentID'] == self.id:
                                    self.action == "move_to_free_neibour_node"
                                    self.moving_away = True
                                    self.remaining_path[0:0] = [free_node1, self.position]  #
                                    # logging.info(f'agent{self.id} --> move to free neighbor node {self.remaining_path}')
                                    self.special_path[0:0] = [free_node1, self.position]


                            else:  # no free neighboring nodes
                                solution[agent_done['AgentID']] = "move_backward_"
                                solution[agent_moving['AgentID']] = "move"
                                Agent_moving_Target = agent_moving['target']

                                agent_done_back_node_is_free = True
                                ##################################################################################
                                back_node = self.get_back_node(agent_done['pos'], agent_moving['pos'])
                                if back_node is None or not map.is_free(back_node):
                                    agent_done_back_node_is_free = False
                                else:
                                    agent_done_back_node_is_free = True

                                ##################################################################################

                                got_to_node2, m_ = map.get_Free_WayNode(agent_done['pos'], agent_moving['pos'],
                                                                        agent_moving['next_next_node'])

                                if got_to_node2 is None:
                                    direction_node = self.get_back_node(agent_done['pos'], agent_moving['pos'])
                                    nearestFreenode = map.get_nearest_free_node_on_right_left_mode(agent_done['pos'],
                                                                                                   direction_node, 1)
                                    # logging.info(f'direct_node{direction_node} --> nearestFreenode {nearestFreenode}: is between {self.is_node_between_two_nodes(nearestFreenode,agent_moving["pos"],agent_done["pos"])}')

                                    if nearestFreenode is not None and not self.is_target_between_two_nodes(
                                            Agent_moving_Target, agent_done['pos'],
                                            nearestFreenode):  # agent_moving['target'] != agent_done['my_last_node'] and agent_moving['target'] not in map.get_neighbors(agent_done['pos'],diagonal=False):
                                        got_to_node2, m_ = map.get_Free_WayNode(agent_done['pos'], agent_moving['pos'],
                                                                                agent_moving['pos'])

                                if got_to_node2 is None:
                                    special_traitement = True
                                    self.special_case = True
                                    self.repeated_nodes.clear()
                                    # self.special_path.clear()

                                if agent_done['AgentID'] == self.id:
                                    self.im_done = False
                                    self.is_last_node = False
                                    self.moving_away = False
                                    i = 1
                                    if got_to_node2 is not None and i == 0:  # *******************
                                        self.moving_backward = True
                                        self.remaining_path[0:0] = [ got_to_node2]  #################,self.positionI addade self.position##############################
                                        # logging.info(f'agent{self.id} done & solved--> move_backward--remaining path: {self.remaining_path}')

                                        self.special_path[0:0] = [got_to_node2]  # ,self.position

                                    else:  # It's a special_case

                                        # logging.info(f'special traitement: Ok')
                                        solution[agent_done['AgentID']] = "replan_path"
                                        node_ = None

                                        if agent_done_back_node_is_free:
                                            node_ = map.get_nearest_free_node_on_right_left_mode(self.position,
                                                                                                 back_node, 1)
                                        else:
                                            node_ = map.get_nearest_free_node_on_right_left_mode(self.position,
                                                                                                 agent_moving['pos'], 1)

                                        # logging.info(f'found_nearest_node: {node_}')
                                        path = self._path_finder.astar_planner(map._graph, agent_done['pos'], node_)
                                        if path is not None and len(path) > 0:
                                            self.moving_backward = False
                                            self.moving_away = True
                                            # print("path", path)
                                            path.pop(0)
                                            path.reverse()
                                            self.remaining_path[0:0] = path  #
                                            self.special_path[0:0] = path
                                            path.reverse()
                                            self.remaining_path[0:0] = path  #
                                            self.special_path[0:0] = path

                                        else:
                                            solution[agent_done['AgentID']] = "wait"

                                            # logging.info(f'solution for agent {agent_done["AgentID"]} changed to Wait')
                                            self.im_done = True
                                            self.is_last_node = True

                                            self.moving_away = False
                                            self.special_case = False
                                            # self.special_path.clear()

                                if agent_moving['AgentID'] == self.id:

                                    self.action == "move_away_nearest_node"
                                    solution[agent_moving['AgentID']] = "move_away_nearest_node"
                                    # Got_free_node = self.move_out_of_the_way(map, agent_done['pos'], agent_done['next_next_node'])

                                    self.special_case = True
                                    self.moving_away = False
                                    node2 = False
                                    mode = 0

                                    nearestnode_befor_target = True  # plan direct path
                                    back_node = self.get_back_node(self.position, agent_done['pos'])

                                    # nearestFreenode = map.get_nearest_free_node_on_right_left_mode(agent_done['pos'], back_node, 1)
                                    node2 = None
                                    if not agent_done_back_node_is_free:
                                        node2 = map.get_nearest_free_node_on_right_left_mode(self.position, back_node,
                                                                                             2)
                                        mode = 1
                                    else:
                                        node2 = map.get_nearest_free_node_on_right_left_mode(self.position,
                                                                                             agent_done['pos'], 2)
                                        mode = 2  # move backwaard and let the other

                                    if node2 is None:
                                        node2 = map.get_nearest_free_node(self.position)

                                    if node2 is not None:
                                        if nearestFreenode is not None and self.is_target_between_two_nodes(
                                                Agent_moving_Target, agent_done['pos'], node2):
                                            nearestnode_befor_target = False

                                    if mode != 0 and not (nearestnode_befor_target and mode == 2):

                                        path1 = self._path_finder.astar_planner(map._copy_graph, self.position, node2)
                                        path2 = self._path_finder.astar_planner(map._copy_graph, node2,
                                                                                Agent_moving_Target)
                                        self.next_target = Agent_moving_Target
                                        if path1 is not None and path2 is not None:
                                            self.moving_backward = False
                                            # self.moving_away = True

                                            path1.pop(0)
                                            #path2.pop(0)

                                            self.remaining_path = path1 + path2  #

                                            self.special_path[0:0] = path1 + path2


                                        else:
                                            solution[agent_moving['AgentID']] = "wait"
                                            # logging.info(f'solution for agent {agent_done["AgentID"]} changed to Wait')
                                            # self.im_done = True      #*************************************************************
                                            # self.is_last_node = True #**************************************************************
                                            self.moving_away = False


                                    elif nearestnode_befor_target and mode == 2:
                                        solution[agent_moving['AgentID']] = "move"

                else:
                    for n in self.neighbors:
                        if n['pos'] == critic_node:
                            # if map.is_free(n['next_next_node']):  # next-next node is free
                            solution[n['AgentID']] = "move"
                            if n['AgentID'] == self.id:
                                self.moving_away = True
                                solved = True
                    for agent in self.neighbors:
                        if agent['pos'] != critic_node:
                            solution[agent['AgentID']] = "wait"

        self.priority_neighbor = priority_agent

        # if self.neighbors[0]['AgentID'] == self.id:
        # logging.info(f'solution is :{solution}')

        for agent in self.neighbors:
            if agent['AgentID'] == self.id:
                self.action = solution[self.id]
                # break

        if self.action == "move_out_of_the_way_intersection":
            self.move_out_of_the_way_intersection(map, self.critic_node, self.critic_node)
            self.moving_backward = True

        if self.action == "move_out_of_the_way":
            self.move_out_of_the_way(map, self.critic_node, self.critic_node)  # self.position

        if self.action == "move_backward":
            self.move_out_of_the_way_intersection(map, self.critic_node, self.critic_node)  # threshold_node

        if self.action == "move_to_node":
            self.move_to_node(map, self.my_move_away_node)  # stay only on time-step in this node

        if self.action == "move_to_AGV_node":
            self.move_to_AGV_node(map, self.my_move_away_node)

        if self.action == "move_to_node_via_critic_node":
            self.move_to_node_via_critic_node(map, self.my_move_away_node)

        if self.action == "move_to_node_and_wait":
            self.move_to_node_and_wait(map, self.my_move_away_node)  # move to this node and stay two time-steps there

        self.next_waypoint = self.remaining_path[0]

    def solve_opposite_conflict(self, map, critic_node: tuple) -> tuple:
        """
        return the id of the agent with the priority to move in the next step
        """
        solution = {}
        candidates = self.neighbors.copy()
        self.critic_node = critic_node
        #if self.neighbors[0]['AgentID'] == self.id:
            #print(f'-----------< solving an opposite_conflict in nodes :  {critic_node}>--------------------')

        priority_agent = None

        if self.neighbors[0]['AgentID'] == self.id:
            self.conflict_agent = self.neighbors[1]['AgentID']
        else:
            self.conflict_agent = self.neighbors[0]['AgentID']

        if priority_agent is None and len(candidates) == 1:
            priority_agent = candidates[0]['AgentID']

        # choose the agent having a free neighbour node>
        if priority_agent is None and len(candidates) == 2:

                got_free_node1 = map.get_right_or_left_free_node(candidates[0]["pos"], candidates[1]["pos"], candidates[1]["next_next_node"])
                got_free_node2 = map.get_right_or_left_free_node(candidates[1]["pos"], candidates[0]["pos"],candidates[0]["next_next_node"])

                if got_free_node1 is None and got_free_node2 is not None:  # give the priority to this agent (it hasn't any free neighboring_node to got to)
                    priority_agent = candidates[0]['AgentID']

                elif got_free_node2 is None and got_free_node1 is not None:
                    priority_agent = candidates[1]['AgentID']

        # <choose the agent with the largest number of Followers >
        if priority_agent is None and len(candidates) == 2:

                    highest_requests_num = []
                    done = False
                    for i in candidates:
                        highest_requests_num.append(i['successors'])
                        # if i["im_done"]:
                        # done=True
                        # break
                    if not done:  # if an agent is done, do not applied this rule
                        indx = highest_requests_num.index(max(highest_requests_num))
                        highest_requests = highest_requests_num[indx]
                        newcandidates = [i for i in candidates if i['successors'] == highest_requests]
                        candidates.clear()
                        candidates.extend(newcandidates)

                    if len(candidates) == 1:  # if only one candidate left then it will have the priority
                        priority_agent = candidates[0]['AgentID']


        if priority_agent is None and len(candidates) == 2:
           # choose the agent having the state 'moving_away' True
           if candidates[0]['moving_away'] and candidates[0]['conflict_agent'] != candidates[1]['AgentID'] and candidates[0]['next_node'] != candidates[0]['my_last_node']:
            # if not candidates[1]['moving_away']:
            priority_agent = candidates[0]['AgentID']
            if candidates[0]['AgentID'] == self.id:
                self.moving_away = False

           elif candidates[1]['moving_away'] and candidates[1]['conflict_agent'] != candidates[0]['AgentID'] and  candidates[1]['next_node'] != candidates[1]['my_last_node']:
              # if not candidates[0]['moving_away']:
              priority_agent = candidates[1]['AgentID']
              if candidates[1]['AgentID'] == self.id:
                 self.moving_away = False

        # if an agent is moving_backward or moving_away and the coflict_agent is the same, then it should continue and give the priority to the coflict_agent
        if priority_agent is None :  # both AGVs have a free neighboring_node or both AGVs don't have any free neighboring_node
            for agent in candidates:
                 if agent['moving_away'] or agent['moving_backward'] and priority_agent is None:  # the other agent will have the priority because it had the last step
                       for n in candidates:
                            if n != agent:
                                 if agent['conflict_agent'] == n['AgentID']:
                                      priority_agent = n['AgentID']
                                      if agent['AgentID'] == self.id:
                                          self.moving_away = False



        # choose the agent with highest num_pos_requests>
        if priority_agent is None and len(candidates) == 2:

                highest_requests_num = []
                for i in candidates:
                    highest_requests_num.append(i['num_pos_requests'])

                indx = highest_requests_num.index(max(highest_requests_num))
                highest_requests = highest_requests_num[indx]
                newcandidates = [i for i in candidates if i['num_pos_requests'] == highest_requests]
                candidates.clear()
                candidates.extend(newcandidates)

                if len(candidates) == 1:  # if only one candidate left then it will have the priority
                    priority_agent = candidates[0]['AgentID']


        # choose the agent with the highest remaining_nodes>
        if priority_agent is None and len(candidates) == 2:  # in case of equality

                remaining_node = []
                for i in candidates:
                    remaining_node.append(i['remaining_nodes'])

                indx = remaining_node.index(min(remaining_node))
                longest_path = remaining_node[indx]
                newcandidates = [i for i in candidates if i['remaining_nodes'] == longest_path]
                candidates.clear()
                candidates.extend(newcandidates)

                if len(candidates) == 1:  # if only one candidate left then it has the priority
                    priority_agent = candidates[0]['AgentID']

                # rule 04: in case every other rule failed , choose the agent with the highest id
                else:
                    #if self.neighbors[0]['AgentID'] == self.id:
                        #logging.info(f'rule 04 failed, we take the agent with the highest id')
                    priority_agent = max([agent['AgentID'] for agent in candidates])


        self.priority_neighbor = priority_agent

        # update variables:
        self.moving_away = False
        self.moving_backward = False
        self.got_priority_last_step = False  #
        self.action=None

        for neighbor in self.neighbors:  # two agents
            if neighbor['AgentID'] == priority_agent:
                self.critic_node = neighbor['pos']  # this is the critic node and priority_agent should move

                break

        prohibited_node = None
        threshould = None
        Got_free_node = True

        for neighbor in self.neighbors:  # two agents
            if neighbor['AgentID'] == priority_agent:
                solution[neighbor['AgentID']] = "move"
                prohibited_node = neighbor['next_next_node']
                threshould = neighbor['pos']

                if priority_agent == self.id:
                    self.action = "move"
        #Got_free_node = self.move_out_of_the_way(map, threshould, prohibited_node)

        # if Got_free_node:
        for n in self.neighbors:  # two agents
            if n['AgentID'] != priority_agent:
                solution[n['AgentID']] = "move_out_of_the_way"
                if n['AgentID'] == self.id:
                    Got_free_node= self.move_out_of_the_way(map, threshould, prohibited_node)
                    if not Got_free_node:
                        self.action = "wait"
                    else:
                        self.action = "move_out_of_the_way"
                    self.next_waypoint = self.remaining_path[0]


        self.next_waypoint = self.remaining_path[0]

        #if self.neighbors[0]['AgentID'] == self.id:
            #logging.info(f'Solution{solution}')

    def post_negotiation(self, map):

        #each agent checks if there are other agents planned the same next node, and then change their action to wait if they do not have a priority

        candidates = []
        conf = False

        agent1 = map.neighbors_agents_stat[self.id]

        # get the agent planned the same next_node with me and negotiate the priority again
        for agent2 in map.neighbors_agents_stat:#map.msg_box.values():
            if agent2['AgentID'] != self.id and agent1["next_node"] != None and agent2["next_node"] != None:
                 if (agent1["next_node"] == agent2["next_node"]):
                    self.got_conflict = True  # my action will depend only on the conflict_resolution process
                    conf = True
                    candidates.append(agent2)  # the other agents

        if conf:
           candidates.append(agent1)  # if there are other agents that has planned the same action as me after solving a conflict

        #candidates_copy = candidates
        all_agents_move = True

        if len(candidates) > 1 :#and map.is_free(candidates[0]["next_node"]):  # == 2:
            for agent in candidates:
                if agent["planned_action"] == "wait" : #and not agent['im_done']:
                    candidates.remove(agent)
                    #candidates_copy.remove(agent)
                    break

        #negotiation process to determine which agent will have priority :
        priority_agent = None
        agent_having_priority=None

        if len(candidates) == 1:
            priority_agent = candidates[0]['AgentID']

        if priority_agent is None and len(candidates) > 1 and all_agents_move :

            if priority_agent is None:
                # if Agent is moving_away or moving _backward, then it will have the priority
                for agent in candidates:

                    if agent['moving_away'] or agent['moving_backward']:
                        # if not candidates[1]['moving_away']:
                        priority_agent = agent['AgentID']
                        agent_having_priority= agent
                        if agent['AgentID'] == self.id:
                            self.moving_away = False
                        break

                # rules2: agent having more followers
                if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

                    highest_requests_num = []
                    done = False
                    for i in candidates:
                        highest_requests_num.append(i['successors'])

                    if not done:  # if an agent is done, do not applied this rule
                        indx = highest_requests_num.index(max(highest_requests_num))
                        highest_requests = highest_requests_num[indx]
                        newcandidates = [i for i in candidates if i['successors'] == highest_requests]
                        candidates.clear()
                        candidates.extend(newcandidates)

                    if len(candidates) == 1:  # if only one candidate left then it will have the priority
                        priority_agent = candidates[0]['AgentID']
                        agent_having_priority = candidates[0]

                    # rule 01: is the agent next-next node is free ?

                if priority_agent is None and len(candidates) > 1:

                    newlist = []
                    for agent in candidates:
                        # if agent['next_next_node'] not in [cand['next_node'] for cand in candidates]:
                        if agent['next_next_node'] not in [candidate['pos'] for candidate in candidates] and int(
                                agent['remaining_nodes']) > 1:
                            newlist.append(agent)
                    if len(newlist) > 0:
                        candidates.clear()
                        candidates.extend(newlist)

                    if len(candidates) == 1:  # if only one candidate left then it will have the priority
                        priority_agent = candidates[0]['AgentID']
                        agent_having_priority = candidates[0]

                # choose the agent with the most position requests
                if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

                    highest_requests_num = []
                    for i in candidates:
                        highest_requests_num.append(i['num_pos_requests'])

                    indx = highest_requests_num.index(max(highest_requests_num))
                    highest_requests = highest_requests_num[indx]
                    newcandidates = [i for i in candidates if i['num_pos_requests'] == highest_requests]
                    candidates.clear()
                    candidates.extend(newcandidates)

                    if len(candidates) == 1:  # if only one candidate left then it will have the priority
                        priority_agent = candidates[0]['AgentID']
                        agent_having_priority = candidates[0]

                #  choose the agent with the longest path
                if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

                    remaining_node = []
                    for i in candidates:
                        remaining_node.append(i['remaining_nodes'])

                    indx = remaining_node.index(max(remaining_node))
                    longest_path = remaining_node[indx]

                    newcandidates = [i for i in candidates if i['remaining_nodes'] == longest_path]
                    candidates.clear()
                    candidates.extend(newcandidates)

                    if len(candidates) == 1:  # if only one candidate left then it will have the priority
                        priority_agent = candidates[0]['AgentID']
                        agent_having_priority = candidates[0]
                    # rule 05: in case every other rule failed , choose the agent with the highest id
                    else:
                        priority_agent = max([agent['AgentID'] for agent in candidates])
                        #agent_having_priority = candidates[candidates.index(priority_agent)]

        if priority_agent is not None and priority_agent != self.id:
              self.action = "wait"
              self.changed_action = True
        self.send_my_data(map)

    def post_coordination(self, map) -> tuple:
        """
        post negotiation to coordinate the planned action
        """
        # only the agent did not participate in the conflict resolution should execute this code
        self.next_waypoint = self.remaining_path[0]

        if not self.got_conflict:

            if self.my_predecessors is not None and len(self.my_predecessors) > 0:
                # logging.info(f'*******<post coordination (agent{self.id} with {len(self.my_predecessors)} predecessors {self.my_predecessors})>************')
                leader = None
                for i in range(len(self.my_predecessors)):
                    if self.is_agent_involved_in_opposite_conflict(map, self.my_predecessors[i]) or (
                    self.is_agent_involved_in_intersection_conflict(map, self.my_predecessors[i])) or (
                    self.my_predecessors[i]["changed_action"]):
                        leader = self.my_predecessors[i]  # i will follow the cation of my predecessors
                        break
                if leader is None:
                    leader = self.my_predecessors[-1]
                # logging.info(f'*******<leader {leader})>************')
                # leader_planned_action = my_predecessors[-1]["planned_action"]

                leader_planned_action = leader["planned_action"]
                # logging.info(f'<leader_action= {leader["planned_action"]})>')
                if leader["planned_action"] == "wait":
                    self.action = "wait"

                    self.changed_action = True
                    # logging.info(f'********* Agent{self.id} changed its action to wait in post_coordination (leader action is wait)')

                else:  # leader will move
                    threshould = None
                    follower = self.get_follower_agent(map, leader["AgentID"])

                    if follower is not None and leader["next_node"] == follower['pos']:
                        is_moving_backward = True
                    else:
                        is_moving_backward = False

                    if is_moving_backward:  # leader["moving_backward"]:
                        agent = self.get_leader_agent(map, self.id)
                        if  agent is not None:
                            threshould = agent['pos']
                            if threshould is not None:
                                Got_free_node = self.move_out_of_the_way(map, threshould, threshould)
                                if not Got_free_node:
                                    self.action = "wait"
                                    self.changed_action = True
                                else:
                                    self.action = "move_out_of_the_way"
                            self.next_waypoint = self.remaining_path[0]

        self.send_my_data(map)

    def plan_last_step_after_negotiation(self, map):

        self.next_waypoint = self.remaining_path[0]
        for agent in map.neighbors_agents_stat:
            if agent['AgentID'] != self.id and self.next_waypoint==agent['pos']  and self.action != "wait":
              if (agent["planned_action"]=="wait") or (agent["next_node"]==agent['pos']):
                 self.action = "wait"
                 self.changed_action = True
                 break
        self.send_my_data(map)

    def move_out_of_the_way(self, map, threshold_node, prohibited_node):
      """
       used when there are more than one critic node
      """
      got_to_node, self.moving_backward = map.get_Free_WayNode(self.position, threshold_node, prohibited_node)

      #logging.info(f'returned node for agent{self.id}: {got_to_node}')
      got_free_node=True

      if got_to_node is None:
         got_to_node, self.moving_backward = map.get_WayNode_include_moveBackward(self.position, threshold_node)
         #logging.info(f'returned node for agent{self.id} with accounting of backward move : {got_to_node}')

      if got_to_node is not None:

        map.new_paths_node[self.id] = got_to_node
        self.im_done = False

        if type(got_to_node) is dict:
            got_to_node = got_to_node['pos']

        if not self.moving_backward:  # got_to_node != self.last_node:
            #logging.info(f'agent {self.id} will move to a free node and return to its pos: {[got_to_node, self.position]}')
            # add the move away path to the remaining path
            self.moving_away = True
        else:
            self.moving_away = False

        # update the path:
        self.remaining_path[0:0] = [got_to_node, self.position]
        #logging.info(f'move_away--remaining path: {self.remaining_path}')
        self.my_move_away_node = got_to_node

      else:#if got_to_node is None or got_to_node==self.position:
          got_to_node = self.position
          self.moving_away = False
          self.moving_backward = False
          #self.remaining_path[0:0] = [got_to_node]
          self.action="wait"
          #logging.info(f'-returned None ---> action of agent {self.id} changed to wait')
          got_free_node= False

      return  got_free_node

    def move_out_of_the_way_intersection(self, map, threshold_node, prohibited_node):
        """
          used when there are more than one critic node
        """
        got_to_node, self.moving_backward = map.get_Free_WayNode(self.position, threshold_node, prohibited_node)
        #logging.info(f'move_out_of_the_way_intersection for the agent{self.id}: {got_to_node}')

        if got_to_node is None:
            got_to_node, self.moving_backward = map.get_Free_WayNode(self.position, threshold_node, threshold_node)

        map.new_paths_node[self.id] = got_to_node
        got_free_node = True

        self.im_done = False
        if got_to_node is not None:
          if not self.moving_backward:  # got_to_node != self.last_node:

            self.moving_away = True

            self.remaining_path[0:0] = [got_to_node, got_to_node , self.position]
            #logging.info(f'--new path: {self.remaining_path}')
            return True

          else:
           self.moving_away = True
           self.remaining_path[0:0] = [got_to_node, self.position]
           #logging.info(f'--new path: {self.remaining_path}')
           self.my_move_away_node = got_to_node


        else:
            got_to_node=self.position
            self.moving_away = False
            self.moving_backward=False
            #self.remaining_path[0:0] = [got_to_node]
            self.action = "wait"
            got_free_node = False

        return got_free_node

    def move_away(self, map, threshold_node):
        """
         used when there are more than one critic node
        """
        got_to_node, self.moving_backward = map.get_move_away_node(self.position, threshold_node)
        #logging.info(f'The returned free node for the agent{self.id}: {got_to_node}')
        self.im_done = False
        map.new_paths_node[self.id]=got_to_node

        if type(got_to_node) is dict:
            got_to_node = got_to_node['pos']

        if not self.moving_backward :#got_to_node != self.last_node:
            # TODO:Try to plan another path
            #logging.info( f'agent {self.id} will move to a free node and return to its pos: {[got_to_node, self.position]}')
            # add the move away path to the remaining path
            self.moving_away = True
        else:
            self.moving_away = False
        #update the path:
        self.remaining_path[0:0] = [got_to_node, self.position]
        #logging.info(f'move_away--remaining path: {self.remaining_path}')
        self.my_move_away_node = got_to_node

    def get_best_AGV_node(self,map, candidates,critic_node):
      priority_agent=None
      node  = None
      agent_= None

      # rule 01: choose the agent having free neighbouring node
      if len(candidates) > 1:
        # rule 01:choose the agent with a free neighbouring node
        got_free_node1 = map.get_right_or_left_free_node(candidates[0]["pos"], critic_node, critic_node)#candidates[1]["pos"], candidates[1]["next_next_node"])  #############
        got_free_node2 = map.get_right_or_left_free_node(candidates[1]["pos"], critic_node, critic_node)#candidates[0]["pos"],candidates[0]["next_next_node"])

        if got_free_node1 is None and got_free_node2 is not None:
           candidates.remove(candidates[0])

        elif got_free_node2 is None and got_free_node1 is not None:
            candidates.remove(candidates[1])

        if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']
      # rule 02: choose the agent with least num_successors
      if len(candidates) > 1:

          num_successors = []
          for i in candidates:
            num_successors.append(i['successors'])
          indx = num_successors.index(min(num_successors))
          smallestRequ = num_successors[indx]
          newcandidates = [i for i in candidates if i['successors'] == smallestRequ]

          candidates.clear()
          candidates.extend(newcandidates)

          if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']
      #rule 03: choose the AGV who not located in my nex_next_node
      if len(candidates) > 1:
          my_next_next_node = None
          for i in self.neighbors:
              if i['AgentID'] == self.id:
                  my_next_next_node = i['next_next_node']
                  break

          for i in candidates:
              if my_next_next_node is not None and i['pos'] == my_next_next_node:
                  candidates.remove(i)
                  break
          if len(candidates) == 1:  # if only one candidate left then it will have the priority
              priority_agent = candidates[0]['AgentID']
      # rule 04: choose the agent with least num_pos_requests
      if len(candidates) > 1:

          requests_num = []
          for i in candidates:
            requests_num.append(i['num_pos_requests'])
          indx = requests_num.index(min(requests_num))
          smallestRequ = requests_num[indx]
          newcandidates = [i for i in candidates if i['num_pos_requests'] == smallestRequ]

          candidates.clear()
          candidates.extend(newcandidates)

          if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']
      # rule 05: choose the agent with the least path
      if len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

            remaining_node = []
            for i in candidates:
                remaining_node.append(i['remaining_nodes'])

            indx = remaining_node.index(min(remaining_node))
            longest_path = remaining_node[indx]

            newcandidates = [i for i in candidates if i['remaining_nodes'] == longest_path]
            candidates.clear()
            candidates.extend(newcandidates)

            if len(candidates) == 1:  # if only one candidate left then it will have the priority
                priority_agent = candidates[0]['AgentID']

            # rule 04: in case every other rule failed , choose the agent with the highest id
            else:
               priority_agent = max([agent['AgentID'] for agent in candidates])

      if len(candidates) == 1:
          priority_agent = candidates[0]['AgentID']

      for agent in candidates:
         if agent['AgentID'] == priority_agent :
              node=agent['pos']
              agent_=agent
              break

      return node,agent_

    def move_backward(self,map):
        #logging.info(f'agent {self.id} will move backward ')
        self.remaining_path.insert(0,self.last_node)
        map.new_paths_node[self.id] = self.last_node
        #logging.info(f'move_backward--remaining path: {self.remaining_path}')
        self.my_move_away_node=self.last_node
        self.moving_away = False
        self.moving_backward = True

    def move_to_node_and_wait(self,map, node_):

        if node_ is not None:
          #map.new_paths_node[self.id] = node_
          self.remaining_path[0:0] = [node_, node_,self.position]#
          #logging.info(f'Agent in {self.position} is moving_and_waiting--remaining path: {self.remaining_path}')
          self.my_move_away_node = node_
          self.moving_away = True
          self.im_done = False

        #else:
          #logging.info(f'-----move_to_node_and_wait--The node is None ')

    def move_to_node(self,map, node_):

        if node_ is not None:
          map.new_paths_node[self.id] = node_
          self.remaining_path[0:0] = [node_, self.position]#
          #logging.info(f'Agent in {self.position} is moving_to_node--remaining path: {self.remaining_path}')
          self.my_move_away_node = node_
          self.moving_away = True
          self.im_done = False

        #else:
          #logging.info(f'-----move_to_node_and_wait--The node is None ')

    def move_to_node_via_critic_node(self,map,my_move_away_node):

        map.new_paths_node[self.id] = my_move_away_node

        self.remaining_path[0:0] = [self.critic_node, my_move_away_node]
        #logging.info(f'Agent{self.id}--remaining path: {self.remaining_path}')
        self.my_move_away_node = my_move_away_node
        self.moving_away = True
        self.im_done = False

    def move_to_AGV_node(self,map, got_to_node):
        """
        define the path of the agent required to move away
        """
        #logging.info(f'agent {self.id} will move to critic_node, AGV_node, critic_node: {[self.critic_node, got_to_node, self.critic_node]} ')
        #add the move away path to the remaining path

        map.new_paths_node[self.id] = got_to_node
        self.im_done = False
        self.remaining_path[0:0] = [self.critic_node, got_to_node]#, self.critic_node
        #logging.info(f'move_to_AGV_node--remaining path: {self.remaining_path}')
        self.moving_away = True

    def get_node_to_remove_replan_path(self,map):

        neighbors = map.get_neighbors(self.position, diagonal=False)
        my_nextnode = None

        for n in neighbors:
            if n not in map._graph.nodes or not map.within_map_size(n):
                neighbors.remove(n)

        for node in self.remaining_path:
            if node != self.position :
                my_nextnode = node
                break

        x, y   = self.position

        if my_nextnode != None:
         x1, y1 = my_nextnode
         if y == y1: # in the same col
           if x1 > x:
               for n in neighbors:
                   x2,y2=n
                   if x2   < x:
                       neighbors.remove(n)
                       break
           elif x1<x:
               for n in neighbors:
                   x2,y2 = n
                   if x2 > x:
                       neighbors.remove(n)
                       break
         elif x==x1: # in the same row
             if y1 > y:
                 for n in neighbors:
                     x2, y2 = n
                     if y2 < y:
                        neighbors.remove(n)
                        break
             elif y1 < y:
                 for n in neighbors:
                     x2, y2 = n
                     if y2 > y:
                        neighbors.remove(n)
                        break

        return  neighbors



    def next_step(self, map) -> None:

        """
        Plan the next step of the agent
        returns: The next node
        """

        # plan the full path if you didn't do that before
        if len(self.target_list) > 0:
            if self.remaining_path is None:
                self.plan_path_to_all_targets(map)
                self.im_done = False


        if self.waiting_steps > 4 and not self.im_done and self.num_TRIES == 0:  # there is deadlock

            self.num_TRIES += 1
            neighbor = map.free_neighboring_node(self.position, self.position)
            if neighbor is not None: # only robots on boarder will plan their path

                neighbors_to_remove = self.get_node_to_remove_replan_path(map)
                #neighbors_to_remove = map.get_all_neighbors(self.position)
                #for node in neighbors_to_remove:
                #    if map.is_free(node) :
                #        neighbors_to_remove.remove(node)

                if self.target in neighbors_to_remove :
                       neighbors_to_remove.remove(self.target )

                if len(neighbors_to_remove) < 4:#:>0

                    path_i = self._path_finder.astar_replan(map._copy_graph, self.position, self.target, neighbors_to_remove)  # neighbors

                    if path_i is not None and len(path_i) > 0:

                        if path_i[0] == self.position and self.position != self.target:
                            path_i.pop(0)

                        self.remaining_path.clear()
                        self.remaining_path.extend(path_i)  #
                        self.num_replanned_paths += 1
                        # print(f' AgentID: {self.id}, waitingtime6, position {self.position}, target {self.target}, planed new path:{path_i}')

        if self.waiting_steps > 5 and not self.im_done and self.num_TRIES == 1:  # there is another deadlock
            # print(f' AgentID: {self.id}, waitingtime 7, {self.position}, {self.target}')

            neighbor = map.free_neighboring_node(self.position, self.position)
            self.num_TRIES += 1
            if neighbor is not None:
                neighbors1 = map.get_neighbors(self.position, diagonal=False)
                for n in neighbors1:
                    if n not in map._graph.nodes or not map.within_map_size(n):
                        neighbors1.remove(n)

                neighbors = []
                for n in neighbors1:
                    if n in map._copy_graph.nodes and not map.is_free(n):
                        neighbors.append(n)

                if len(neighbors) < 4:

                    path_i = self._path_finder.astar_replan(map._copy_graph, self.position, self.target_list[0], neighbors)  # neighbors
                    if path_i is not None and len(path_i) > 0:

                        if path_i[0] == self.position and self.position != self.target_list[0]:
                            path_i.pop(0)
                        self.remaining_path.clear()
                        self.remaining_path.extend(path_i)  #
                        self.num_replanned_paths += 1
                        # print(f' AgentID: {self.id}, waitingtime7, position {self.position}, target {self.target}, planed new path:{path_i}')

        if self.waiting_steps > 6 and not self.im_done and self.num_TRIES == 2:  # there is another deadlock
            # print(f' AgentID: {self.id}, waitingtime 8, {self.position}, {self.target}')

            neighbors = map.get_neighbors(self.position, diagonal=False)
            neighbor = map.free_neighboring_node(self.position, self.position)
            self.num_TRIES += 1

            if neighbor is not None and neighbor in neighbors:
                neighbors.remove(neighbor)

            for n in neighbors:
                if n not in map._graph.nodes or not map.within_map_size(n) or map.is_free(n):
                    neighbors.remove(n)

            path_i = self._path_finder.astar_replan(map._copy_graph, self.position, self.target_list[0],
                                                    neighbors)  # neighbors
            if path_i is not None and len(path_i) > 0:

                if path_i[0] == self.position and self.position != self.target_list[0]:
                    path_i.pop(0)
                self.remaining_path.clear()
                self.remaining_path.extend(path_i)  #
                self.num_replanned_paths += 1
                # print(f' AgentID: {self.id}, waitingtime6, position {self.position}, target {self.target}, planed new path:{path_i}')

        if self.waiting_steps > 7 and not self.im_done and self.num_TRIES == 3:  # there is deadlock
            self.num_TRIES += 1
            if self.remaining_path is not None and len(self.remaining_path) > 1:
                if self.remaining_path[0] != self.position and self.remaining_path[0] != self.target_list[ 0]:  # self.target
                    path_i = self._path_finder.astar_replan(map._copy_graph, self.position, self.target_list[0],[self.remaining_path[0]])  # neighbors
                    if path_i is not None and len(path_i) > 0:
                        if path_i[0] == self.position:
                            path_i.pop(0)
                        self.remaining_path.clear()
                        self.remaining_path.extend(path_i)  #
                        self.num_replanned_paths += 1
                        # self.waiting_step = 0

        if self.waiting_steps > 8 and not self.im_done and self.num_TRIES == 4:  # try for the last time to solve it.
            self.num_TRIES = 0
            self.waiting_step = 5  # to start from the first try


            neighbor = map.free_neighboring_node(self.position, self.position)

            # if neighbor is None: # no free neighboring node
            node_ = map.get_nearest_free_node(self.position)
            #print(f'Found random node ***************: {node_}')
            if node_ is not None:

                path = self._path_finder.astar_planner(map._graph, self.position, node_)
                if path is not None and len(path) > 0:
                    self.moving_backward = False
                    self.moving_away = True
                    if path[0] == self.position and self.position != self.target_list[0]:
                        path.pop(0)
                    self.path = path
                    self.remaining_path.clear()
                    self.remaining_path.extend(path)  #
                    self.num_replanned_paths += 1

                # remove my next node from the graph (changed: self.position and remove comment from: self.remaining_path.clear())
                path_i = self._path_finder.astar_replan(map._copy_graph, node_, self.target_list[0], [self.remaining_path[len(self.remaining_path) - 2]])
                if path_i is None:
                    path_i = self._path_finder.astar_planner(map._graph, node_, self.target_list[0])
                if path_i is not None and len(path_i) > 0:
                    if path_i[0] == node_ and self.position != self.target_list[0]:  # self.position
                        path_i.pop(0)

                    self.remaining_path.extend(path_i)  #


        if self.remaining_path is None or len(self.remaining_path) == 0:
            self.im_done = True
            self.target_list = [self.position]
            self.remaining_path = [self.position]
            self.next_waypoint = self.position  # need it to know

        else:
            if len(self.remaining_path) >= 1:
                # calculate the next target and next waypoint
                if self._current_target_id < len(self.target_list):
                    self.next_target = self.target_list[self._current_target_id]

                self.next_waypoint = self.remaining_path[0]
                if self.next_waypoint is None:
                    self.next_waypoint = self.position
                    self.remaining_path.pop(0)
                    self.remaining_path.insert(0, self.position)

                # if you get to the next target
                if self.next_waypoint == self.next_target:
                    if self._current_target_id + 1 < len(self.target_list):
                        self._current_target_id += 1

    def move(self, map, sim_time, time_lapsed: float = 0):
        #self._current_target_id = 0      #######################

        if not self.targetReached :
            self.steps += 1
        if self.position == self.target:
            self.targetReached = True

        for agent in map.neighbors_agents_stat:#.msg_box.values():
            # if agent is not None and agent_me is not None and agent['AgentID'] !=agent_id:
          if agent['AgentID'] != self.id and (self.remaining_path[0]==agent['pos'])  and (self.action != "wait") :
            if (agent["planned_action"]=="wait") or (agent["next_node"]==agent['pos'] ):
                self.action = "wait"
                self.changed_action = True
                self.send_my_data(map)
                break

        # logging.info(f'##############################################')
        # wait for this step
        # logging.info(f'Agent {self.id} --->action:{self.action} (changed:{self.changed_action})')

        self.delay_steps -= 1

        if self.delay_steps == 0:
            #if self.has_delayed and not self.im_done:
            #   self.steps += 1
            self.has_delayed = False

        if len(self.remaining_path) > 0:
            self.next_target   = self.target_list[self._current_target_id]
            self.next_waypoint = self.remaining_path[0]


        if self.action == "wait":
            self.wait()
            self.waiting_steps = self.waiting_steps + 1  #self.waiting_steps +=1
            # logging.info(self.position)


        else: # your planned action is move

           if (self.next_waypoint is None):
              # pyautogui.alert(text='Agent' + str(self.id) + ' with action ' + str(self._position) + ' is trying to reach a None node' , title='Moving to None node',
              #     button='OK')
              self.action="wait"
              self.wait()
              self.waiting_steps = self.waiting_steps + 1


           else:
                self.waiting_steps = 0
                self.num_TRIES = 0
                #if self.last_node != self.position:
                #self.steps += 1


                self.face_to(self.next_waypoint)
                # logging.info(f'{self._position}-->{self.next_waypoint}')
                self.last_node = self._position
                self.my_pos_will_be_free = True
                self.my_move_away_node=None

                #update the graph : set the current pos as free
                map._graph.nodes[self._position]["obstacle"] = False
                map._graph.nodes[self._position]["agent"]    = None
                map._graph.nodes[self._position]["state"]    = 'free_space'

                self._position = self.next_waypoint

                #set_as_free(self._position)
                agent=self
                #map.add_agent_to_map(agent)
                map._graph.nodes[self._position]["obstacle"] = False
                map.graph.nodes[self._position]["agent"]     = agent
                map._graph.nodes[self._position]["state"]    = 'agent'

                if self.is_last_node: # if this node is the last one in the path, then im_done
                    self.is_last_node = False
                    self.im_done   = True
                    self.targetReached = True
                    self.has_delayed = False


                    #map.add_agent_to_list_agents_done(self.position)

                #update self.remaining_path by removing the waypoint from it
                if self.next_waypoint in self.remaining_path:
                   self.all_repeated_nodes.append(self.next_waypoint)
                   if not self.special_case:
                       self.repeated_nodes.append(self.next_waypoint)

                   if self.special_case  and self.next_waypoint in self.special_path:
                       self.special_path.remove(self.next_waypoint)

                   if len(self.special_path) ==0:
                       self.special_case = False

                   self.remaining_path.remove(self.next_waypoint)
                   if len(self.remaining_path) == 0: #
                       self.is_last_node = True

        #self.storage_container.append( {'sim_time': round(sim_time, 2), 'steps': self.steps, 'num_targets': len(self.target_list),'num_conflicts': self.num_conflicts})

        if not self.started:
            self.target = self.next_target
            self.started = True
        #self.action         = None
        self.neighbors      = []
        self.got_conflict   = False
        self.got_opposite_conflict=False
        # = None
        self.changed_action = False
        self.special_case   = False
        #self.moving_backward= False
        #self.got_priority_last_step= False
        #logging.info(f'--------------------------------------------')

    def plan(self,map)->list:
        """
        method for planning the agent's path to its targets
        :param path_finder: the astar planner
        :return:
        """
        # plan a path to the next target
        target = self.target_list[self._current_target_id]
        self.path = self._path_finder.astar_planner(map.graph, self.position, target)
        if self.path[0] == self.position:
            self.path.pop(0)

        map.set_as_path(self.path)


        return self.path

    def plan_path_to_all_targets(self, map)->None:
        """
        method for planning the agent's path to its targets
        :param path_finder: the astar planner
        :return:
        """
        # the initial waypoints before any plan is made
        #logging.info(self.target_list)
        general_waypoints = [self.position] + [target for target in self.target_list ]
        self.remaining_path = []
        #planning the full path including the targets
        self.last_node = self.position

        for i in range(0,len(general_waypoints)-1):
           #calculate obstacle free path between two general waypoints
           path_i = self._path_finder.astar_planner(map._copy_graph, general_waypoints[i], general_waypoints[i+1])
           if path_i is not None :
            # don't allow for duplicates in the path
            if general_waypoints[i] in self.remaining_path:
                path_i.pop(0)

            # concatenate this portion of the path to the full path
            self.remaining_path+=path_i
        # store the full path for later use
        self.path = self.remaining_path

    def wait(self):
        pass

    def face_to(self,position):

        row,col = position
        if row < self.row and col == self.col:
            self.orientation = 1        # face up
        elif row > self.row and col == self.col:
            self.orientation = 3        # face down
        elif row == self.row and col > self.col:
            self.orientation = 0        # face right
        elif row == self.row and col < self.col:
            self.orientation = 2        # face left

    @property
    def target_id(self):
        """
        return the current target id
        :return:
        """
        return self._current_target_id

    @property
    def remaining_nodes(self):
        """
        return the current target id
        :return:
        """
        return len(self.remaining_path)


def smart_agent_generator(args,my_map) -> Callable:
    """Create a random agent generator
    :returns: A generator function
    """
    def generator(position: Tuple[int, int]):
        return SmartAgent(None,my_map, position=position, num_targets=args.num_targets)

    return generator