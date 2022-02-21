""" A smart Agent"""

import logging
import time
import  ctypes
import numpy as np

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

        # create targets automatically
        for n in range(0, self.num_targets):
            target = my_map.get_random_free_node()
            if self.num_targets > 0: # if num_targets == 0, then you should add targets manually
                self.target_list.append(target)  #make it a comment if you want to set the target pos manually

        # initial path info
        self.path                 = None
        self.remaining_path       = None
        self._current_target_id   = 0
        self._current_waypoint_id = 0
        self.moving_away   = False
        self.next_target   = None
        self.next_waypoint = None
        self.critic_node   = None

        self.my_move_away_node      = None
        self.got_priority_last_step = False
        self.last_node              = None
        self.my_pos_will_be_free    = False
        self.moving_backward        = False

        # list of neighbors
        self.neighbors         = []
        self.priority_neighbor = None
        self.action            = None#"wait"
        self.had_conflict      = False
        self.conflict_agent=None
        # DATA TO BE STORED
        self.steps             = 0
        self.im_done           = False
        self.is_last_node         = False
        self.num_conflicts     = 0
        self.storage_container = [{'sim_time':0,'steps':self.steps,'num_targets':len(self.target_list),'num_conflicts':self.num_conflicts}]


    def send_my_data(self,map:Map):
        """
        send the agent data to the map object
        in order to communicate with other agents
        """
        num_pos_requests,num_successors = self.num_pos_requests_and_successors(map, self.position)

        data={"AgentID": self.id,
              "im_done":self.im_done,
              "pos": self.position,
              "remaining_nodes": len(self.remaining_path)if not self.im_done else 0,
              "next_node": self.remaining_path[0] if len(self.remaining_path)>0 else self.target_list[-1],#TODO change it to None
              "next_next_node": self.remaining_path[1] if len(self.remaining_path)>1 else self.target_list[-1] ,
              "num_pos_requests":num_pos_requests,
              "successors": num_successors,
              "had_conflict":self.had_conflict,
              "moving_away":self.moving_away,# variable : True/False
              "moving_backward":self.moving_backward,
              "conflict_agent": self.conflict_agent, # node
              "my_last_node":self.last_node,
              "got_priority_last_step":self.got_priority_last_step,
              "my_pos_will_be_free":self.my_pos_will_be_free,
              "planned_action": self.action}
        map.msg_box[self.id] = data


    def is_agent_implied_in_opposite_conflict(self,map,agent):

        return_=False
        for msg in map.msg_box.values():
            if msg["AgentID"] == agent["AgentID"]:
                continue
            else:
                if (msg["next_node"] == agent["pos"]) and (agent["next_node"] == msg["pos"]):
                    return_= True
                    break

        return  return_

    def is_agent_implied_in_intersection_conflict(self,map,agent):

        return_=False
        for msg in map.msg_box.values():
            if msg["AgentID"] == agent["AgentID"]:
                continue
            else:
                 if (msg["next_node"] == agent["next_node"]) :
                    return_= True
                    break

        return  return_

    def num_pos_requests_and_successors(self, map, position:tuple)->tuple:
        """
        Find the agents requesting this position in the next or the next-next step and successors
        args: position:(tuple) the coordinates of the node ( row,col)
        return:   num_pos_requests_and_successors
        """
        pos_requests = 0
        pos_requests_list = []
        num_successors= 0

        # successors + other agents having 'next_next_node' = my_pos
        for agent in map.msg_box.values():
            if ( (agent['next_node'] == position) or (agent['next_next_node'] == position) ) and agent['AgentID'] != self.id:
                pos_requests +=1
                pos_requests_list.append(agent)

            #if (agent['next_node'] == position)  and agent['AgentID'] != self.id:
                #pos_requests += agent['num_pos_requests']

        # successors
        #for agent in map.msg_box.values():
            #if agent['AgentID'] != self.id:
                #continue
            #elif agent['next_node'] == position:
                #num_successors += 1
                #num_successors += agent['successors']
        succe=self.get_successors(map)
        num_successors = len(succe)

        return pos_requests,num_successors

    def get_successor_agent(self, map,agent_id):
        """
         return the agent having my pos as it next-node
        """
        concerned_Agent=None
        successor=None
        if agent_id is not None:
         for agent in map.msg_box.values():
            if agent['AgentID'] == agent_id:
                concerned_Agent = agent
                break

         for agent in map.msg_box.values():
           if agent is not None and concerned_Agent is not None:
             if (agent['next_node'] == concerned_Agent['pos']) and (agent['pos'] != concerned_Agent['next_node'])and agent['AgentID'] != agent_id:
                successor = agent
                break

        return successor

    def get_predecessor_agent(self, map, agent_id):
        """
          return the agent located in my next-node
        """
        agent_ = None
        predecessor = None
        for agent in map.msg_box.values():
            if agent['AgentID'] == agent_id :
                agent_ = agent
                break

        for agent in map.msg_box.values():
          #if agent is not None and agent_ is not None and agent['AgentID'] !=agent_id:
            if (agent['pos'] == agent_['next_node']) and agent['AgentID'] != agent_id and (agent['next_node'] != agent_['pos']):
                predecessor = agent

        return predecessor

    def get_predecessors(self,map):
        """
        return the list of agent in front of me
        """
        _predecessors =[]
        predecessor= self.get_predecessor_agent(map,self.id)
        if predecessor is not None:
           _predecessors.append(predecessor)
           agent_id =predecessor['AgentID']

        while predecessor is not None and predecessor['AgentID'] !=self.id: #and predecessor not in _predecessors: # get the predecessor of my predecessor
            predecessor = self.get_predecessor_agent(map, agent_id)
            if predecessor is not None and predecessor not in _predecessors:
                _predecessors.append(predecessor)
                agent_id =predecessor['AgentID']

        return _predecessors

    def get_successors(self,map):
        """
        return the list of agent in front of me
        """
        _successors =[]
        successor= self.get_successor_agent(map,self.id)
        if successor is not None:
           _successors.append(successor)
           agent_id = successor['AgentID']

        while successor is not None and successor['AgentID'] !=self.id:

            successor = self.get_successor_agent(map, agent_id)
            if successor is not None and successor not in _successors:
                _successors.append(successor)
                agent_id =successor['AgentID']

        return _successors

    # this function will update the neighbors list of this agent
    def get_neighbors(self, map):
            neighbors = []
            my_next_node = None
            if len(self.remaining_path) > 0:
                my_next_node = self.remaining_path[0]

            for msg in map.msg_box.values():
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

            for msg in map.msg_box.values():
                if msg["next_node"] != None  :#and not self.is_agent_implied_in_opposite_conflict(map,msg):
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

            for msg in map.msg_box.values():
                if msg["next_node"] != None:
                    if (msg["next_node"] == self._position and msg["pos"] == my_next_node) or \
                        msg["AgentID"] == self.id:
                        neighbors.append(msg)
            return neighbors

    def is_free(self,node,neighbors):
        ret=True
        for agent in neighbors:
            if agent['pos']==node:
                ret = False
                break

        return ret

    def get_agent_object(self,agent_id):
        agent=None
        for msg in map.msg_box.values():
            if msg["AgentID"] == agent_id:
                agent=msg
                break
        return  agent

    def check_for_conflict(self,map):

        #if len(self.remaining_path) >= 1:
        my_next_node = self.remaining_path[0]#self.next_waypoint
        #else:
        #my_next_node = self.target_list[-1]

        next_nodes = []
        is_opposite_conflict = False
        for msg in map.msg_box.values():

            if msg["AgentID"] == self.id:
                continue
            else:
                #1-opposite conflict (two conflict nodes)
                if (msg["next_node"] == self.position) and (my_next_node == msg["pos"]):
                    self.neighbors = self.get_opposite_conflict_neighbors(map)
                    next_nodes.append(msg["next_node"])
                    next_nodes.append(my_next_node)
                    is_opposite_conflict = True
                    return True,next_nodes

                # -intersection conflict
                elif (msg["next_node"] == my_next_node):
                  if not is_opposite_conflict:
                    self.neighbors = self.get_intersection_conflict_neighbors(map)
                    # remove the agents having opposite_conflict then check if len (self.neighbors)>1
                    for agent in self.neighbors :
                      if self.is_agent_implied_in_opposite_conflict(map,agent):
                          self.neighbors.remove(agent)
                    if len (self.neighbors) > 1:
                         next_nodes.append(my_next_node)
                         return True,next_nodes
                    else:
                       continue


        self.neighbors = []
        return False,None

    def handle_conflicts(self, map):
        if len(self.remaining_path) >= 1:
            my_next_node = self.next_waypoint
        else:
            my_next_node = self.target_list[-1]
        # solve conflicts for all agents
        is_conflict, critic_node = self.check_for_conflict(map)

        if is_conflict:

            self.had_conflict = True

            if len(critic_node) == 1:
                _node = critic_node[0]
                if type(_node) is list:
                    self.critic_node = _node[0]
                else:
                    self.critic_node = _node
                # if not map.occupied(self.critic_node):
                self.solve_intersection_conflict(map, self.critic_node)

            else:

               self.solve_opposite_conflict(map, critic_node)


        else: #check if there is successor agent has the longest path: if yes, move away and let him pass
            self.my_pos_will_be_free = True
            self.action = "move"
            self.had_conflict = False
            #if not self.is_agent_implied_in_opposite_conflict(map,self.id) and not self.is_agent_implied_in_intersection_conflict(map, self.id):
             #for msg in map.msg_box.values():
              #if msg["AgentID"] == self.id :
            successor= self.get_successor_agent(map,self.id)
            if successor is not None and successor["AgentID"] != self.id :
                if not self.is_agent_implied_in_opposite_conflict(map,successor) and not self.is_agent_implied_in_intersection_conflict(map, successor):
                 if (successor["next_node"] == self.position) and (successor["next_next_node"] == my_next_node) and successor["remaining_nodes"] > len(self.remaining_path):
                    # I should need to move away when I find a free node and let successor pass
                    got_free_node = map.get_Up_Down_free_node(self.position,successor['pos'],successor['pos'])

                    for msg in map.msg_box.values():
                        if msg["next_node"] ==got_free_node:# check if this node is the node of another agent
                            got_free_node=None
                            break

                    if got_free_node is not None:
                       self.action = "let_agent_pass"
                       self.remaining_path[0:0] = [got_free_node, self.position]

                       logging.info(f'move_out_away--and the agent{msg["AgentID"]} pass')
                       logging.info(f'my path:{self.remaining_path}')

    #Rules defining the priority
    def choose_agent_having_next_next_node_free(self, candidates:tuple):
        # rule 00: is the agent next-next node is free ?
        newlist = []
        priority_agent=None
        if candidates[0]==self.id:
           logging.info(f'condidates before: {candidates}')

        for agent in candidates:
            if agent['next_next_node'] not in [cand['next_node'] for cand in candidates]:
                if agent['next_next_node'] not in [candidate['pos'] for candidate in candidates]:
                    newlist.append(agent)

        if len(newlist) > 0:
            candidates.clear()
            candidates.extend(newlist)

        if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']
        logging.info(f'condidates after: {candidates}')
        return priority_agent, candidates

    def is_agent_got_priority_last_step(self,candidates:tuple):
        priority_agent=None

        for agent in candidates:
            if agent["got_priority_last_step"]:
                if candidates[0]['AgentID'] == self.id:
                   logging.info(f'the agent{agent["AgentID"]} got priority last step and it will have the priority this step')
                priority_agent = agent['AgentID']
                if priority_agent == self.id: # it is me
                   self.got_priority_last_step = False #
                break

        return  priority_agent

    def choose_agent_having_largest_num_successors(self,candidates:tuple):
        highest_requests_num = []
        priority_agent =None
        done = False
        for i in candidates:
            highest_requests_num.append(i['successors'])
            if i["im_done"]:
                done = True
                break

        if not done:  # if an agent is done, do not applied this rule
            indx = highest_requests_num.index(max(highest_requests_num))
            highest_requests = highest_requests_num[indx]
            newcandidates = [i for i in candidates if i['successors'] == highest_requests]
            candidates.clear()
            candidates.extend(newcandidates)

        if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']

        return  priority_agent,candidates

    def choose_agent_having_largest_num_request(self,candidates:tuple):
        highest_requests_num = []
        priority_agent = None
        for i in candidates:
            highest_requests_num.append(i['num_pos_requests'])

        indx = highest_requests_num.index(max(highest_requests_num))
        highest_requests = highest_requests_num[indx]
        newcandidates = [i for i in candidates if i['num_pos_requests'] == highest_requests]
        candidates.clear()
        candidates.extend(newcandidates)

        if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']

        return priority_agent, candidates

    def choose_agent_having_longest_path(self,candidates:tuple):
        remaining_node = []
        priority_agent=None

        for i in candidates:
            remaining_node.append(i['remaining_nodes'])

        indx = remaining_node.index(max(remaining_node))
        longest_path = remaining_node[indx]

        newcandidates = [i for i in candidates if i['remaining_nodes'] == longest_path]
        candidates.clear()
        candidates.extend(newcandidates)

        if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']

        return priority_agent, candidates

    def choose_agent_with_highest_ID(self,candidates:tuple):
        return max([agent['AgentID'] for agent in candidates])

    def define_priority_intersection_conflict(self,candidates:tuple):

        priority_agent = None
        if candidates[0] == self.id:
            logging.info(f'rule01: is_agent_got_priority_last_step ?')

        if priority_agent is None and len(candidates) > 1:
            priority_agent,condidates_=self.is_agent_got_priority_last_step(candidates)
            candidates.clear()
            candidates.extend(condidates_)

        if priority_agent is None and len(candidates) > 1:
            if candidates[0] == self.id:
                logging.info(f'rule01: failed ?')
                if candidates[0] == self.id:
                    logging.info(f'rule02: choose_agent_having_next_next_node_free ?')
            priority_agent, condidates_ = self.choose_agent_having_next_next_node_free(candidates)
            candidates.clear()
            candidates.extend(condidates_)

        if priority_agent is None and len(candidates) > 1:
            logging.info(f'rule02: failed ?')
            if candidates[0] == self.id:
                logging.info(f'rule03: choose_agent_having_largest_num_successors ?')
            priority_agent, condidates_ = self.choose_agent_having_largest_num_successors(candidates)
            candidates.clear()
            candidates.extend(condidates_)

        if priority_agent is None and len(candidates) > 1:
            logging.info(f'rule03: failed ?')
            if candidates[0] == self.id:
                logging.info(f'rule04: choose_agent_having_largest_num_request ?')
            priority_agent, condidates_ = self.choose_agent_having_largest_num_request(candidates)
            candidates.clear()
            candidates.extend(condidates_)

        if priority_agent is None and len(candidates) > 1:
            logging.info(f'rule04: failed ?')
            if candidates[0] == self.id:
                logging.info(f'rule05: choose_agent_having_longest_path ?')
            priority_agent, condidates_= self.choose_agent_having_longest_path(candidates)
            candidates.clear()
            candidates.extend(condidates_)

        if priority_agent is None and len(candidates) > 1:
            logging.info(f'all rules: failed ?')
            if candidates[0] == self.id:
                logging.info(f'rule05: choose_agent_with_highest_ID ?')
            priority_agent= self.choose_agent_with_highest_ID(candidates)


        return priority_agent

    def solve_intersection_conflict(self, map, critic_node:tuple)->tuple:
        """
         return the id of the agent with the priority to move in the next step
        """
        solution = {}
        is_critic_node_free = True
        id_critic = None
        self.critic_node = critic_node
        candidates = self.neighbors.copy()

        if self.neighbors[0]['AgentID'] == self.id:
          logging.info(f'-----------<solving an intersection_conflict in node {critic_node} >--------------------')
        priority_agent = None

        if self.neighbors[0]['AgentID'] == self.id:
           logging.info(f'intersection_conflict candidates:{candidates}')
           logging.info(f'---Determine which agent will have priority for the intersection_conflict---')

        for agent in candidates:
            if agent["got_priority_last_step"] and int(agent['remaining_nodes']) > 1: #or agent["moving_away"]:
                if self.neighbors[0]['AgentID'] == self.id:
                   logging.info(f'agent{agent["AgentID"]} got priority last step and it will have the priority this step')
                priority_agent = agent['AgentID']
                if priority_agent == self.id: # it is me
                   self.got_priority_last_step = False #
                break

        if priority_agent is None and len(candidates) > 1 :

            if self.neighbors[0]['AgentID'] == self.id:
               logging.info(f'rules one checking .... neighbors list length = {len(self.neighbors)}')
               logging.info(f'<rule 01 : the next-next node of an agent is free ? > : under checking...')


            if priority_agent is None and len(candidates) > 1:

              #rule 01: is the agent next-next node is free ?
              newlist = []
              for agent in candidates:
                #if agent['next_next_node'] not in [cand['next_node'] for cand in candidates]:
                if agent['next_next_node'] not in [candidate['pos'] for candidate in candidates] and int(agent['remaining_nodes']) > 1:
                    newlist.append(agent)
              if len(newlist) > 0:
                candidates.clear()
                candidates.extend(newlist)

              if len(candidates)==1: # if only one candidate left then it will have the priority
                priority_agent = candidates[0]['AgentID']
                if self.neighbors[0]['AgentID'] == self.id:
                   logging.info(f'rule 01: True for agent{priority_agent}')

              # rule 02: choose the agent with successors
            if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02
                   if self.neighbors[0]['AgentID'] == self.id:
                       logging.info(f'<rules 2: choose the agent successor agents > : under checking... ')
                   highest_requests_num = []
                   done=False
                   for i in candidates:
                       highest_requests_num.append(i['successors'])
                       #if i["im_done"]:
                           #done=True
                           #break
                   if not done: # if an agent is done, do not applied this rule
                     indx = highest_requests_num.index(max(highest_requests_num))
                     highest_requests = highest_requests_num[indx]
                     newcandidates = [i for i in candidates if i['successors'] == highest_requests]
                     candidates.clear()
                     candidates.extend(newcandidates)

                   if len(candidates) == 1:  # if only one candidate left then it will have the priority
                       priority_agent = candidates[0]['AgentID']

                       if self.neighbors[0]['AgentID'] == self.id:
                           logging.info(f'priority rule 02: True for agent{priority_agent}')

            # rule 03: choose the agent with the most position requests
            if priority_agent is None and len(candidates) > 1 : # in case of equality in rule 01 apply rule 02

                if self.neighbors[0]['AgentID'] == self.id:
                    logging.info(f'rule 01 : failed')
                    logging.info(f'<rules 2: choose the agent with the most position requests> : under checking... ')
                highest_requests_num = []
                for i in candidates:
                    highest_requests_num.append(i['num_pos_requests'])

                indx             = highest_requests_num.index(max(highest_requests_num))
                highest_requests = highest_requests_num[indx]
                newcandidates    = [i for i in candidates if i['num_pos_requests'] == highest_requests]
                candidates.clear()
                candidates.extend(newcandidates)

                if len(candidates) == 1:  # if only one candidate left then it will have the priority
                    priority_agent = candidates[0]['AgentID']

                    if self.neighbors[0]['AgentID'] == self.id:
                       logging.info(f'priority rule 02: True for agent{priority_agent}')

            #rule 04 : choose the agent with the longest path
            if priority_agent is None and len(candidates) > 1: # in case of equality in rule 01 apply rule 02
                    if self.neighbors[0]['AgentID'] == self.id:
                       logging.info('rule 02 failed')
                       logging.info(f'< rules 3: choose the agent with the longest path > : under checking... ')
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
                        if self.neighbors[0]['AgentID'] == self.id:
                           logging.info(f'priority rule 03: True for agent{priority_agent}')


                    #rule 05: in case every other rule failed , choose the agent with the highest id
                    else:
                        priority_agent = max([agent['AgentID'] for agent in candidates])
                        if self.neighbors[0]['AgentID'] == self.id:
                           logging.info(f'rule 03 failed--we take the agent with the highest id : (agent{priority_agent})')

        prohibited_node=None
        threshold_node = None
        # update variables:
        self.moving_away = False
        self.moving_backward = False
        self.got_priority_last_step = False #
        agent_having_priority=None
        for agent in self.neighbors:
            if agent['AgentID'] == priority_agent:
                agent_having_priority=agent
                break

        if len(self.neighbors)   == 4 : # 4 AGVs in an intersection
          moveAGVnode = None
          if self.is_free(critic_node, self.neighbors):
            for n in self.neighbors :
                if n['pos'] == agent_having_priority['next_next_node'] :#and n['AgentID']!= agent_having_priority['AgentID']

                          # get a free node without moving back
                          got_free_node1 = map.get_Up_Down_free_node(n['pos'], critic_node,critic_node)

                          if self.neighbors[0]['AgentID'] == self.id:
                              logging.info(f'get_Up_Down_free_node return--{got_free_node1}')

                          if got_free_node1 is None :
                              solution[n['AgentID']] = "move_to_AGV_node"
                              solution[agent_having_priority['AgentID']] = "wait"

                              #chose the node of the agent to move to
                              condidate_=self.neighbors.copy()
                              condidate_.remove(n)        # remove my self from the neighbors
                              condidate_.remove(agent_having_priority) # remove the agent having the priority from the list
                              moveAGVnode,agent_= self.get_best_AGV_node(map,condidate_,critic_node)

                              #############################################################################################################
                              #TODO: add solution to agent : move_to_node_and_wait
                              got_free_node3,_= map.get_move_out_away_node(agent_['pos'], critic_node,critic_node)
                              solution[agent_['AgentID']] = "move_to_node_and_wait"
                              if agent_['AgentID'] == self.id:  #it is me
                                 self.my_move_away_node = got_free_node3
                              #############################################################################################################

                              if n['AgentID'] == self.id:  # to update the AGV node o move to
                                 self.my_move_away_node= moveAGVnode
                              if agent_having_priority['AgentID']==self.id:
                                  self.got_priority_last_step=True

                              if self.neighbors[0]['AgentID'] == self.id:
                                logging.info(f'agent{agent_having_priority["AgentID"]} having priority should wait')
                                logging.info(f'agent{n["AgentID"]} should move to AGV node {moveAGVnode}')


                          else:#if (got_free_node1 != msg['next_node'] for msg in map.msg_box.values() ):
                              solution[agent_having_priority['AgentID']] = "move"
                              solution[n['AgentID']]                     = "move_to_node_and_wait"
                              if n['AgentID'] == self.id:  # it is me
                                 self.my_move_away_node = got_free_node1

                else:
                  #solution[agent_having_priority['AgentID']] = "move"
                   solution[n['AgentID']] = "wait"
                   if n['AgentID'] == self.id:
                       self.action== "wait"
          else:
              got_free_node = None
              solved = False
              if self.neighbors[0]['im_done'] or self.neighbors[1]['im_done'] or self.neighbors[2]['im_done']or self.neighbors[3]['im_done']:
                  for agent in self.neighbors:
                      if agent['im_done']:  # self.im_done :#and self.action=="wait" : # :
                          # if not is_critic_node_free : #critic node is not free
                          # if id_critic ==self.id: #i need to move out away
                          if agent['AgentID'] == self.id:  #
                              self.im_done = False
                              self.is_last_node = False

                              solution[agent['AgentID']] = "move"

                              got_to_node= map.free_neighboring_node(self.position,self.neighbors)

                              if got_to_node is not None:
                                  self.remaining_path[0:0] = [got_to_node]
                                  logging.info(f'agent{self.id} done --> move_away--remaining path: {self.remaining_path}')
                              else:
                                  solution[agent['AgentID']] = "wait"

                      else:
                          solution[agent['AgentID']] = "wait"
              else:
                  for n in self.neighbors:
                      if n['pos'] == critic_node:
                          solution[n['AgentID']] = "move"
                          if n['AgentID'] == self.id:
                              self.moving_away = True
                      else:
                          solution[agent['AgentID']] = "wait"


        elif len(self.neighbors) == 3 :
            is_free=True
            for n in self.neighbors:
               if n['AgentID'] != agent_having_priority['AgentID'] :
                   if agent_having_priority['next_next_node'] == n['pos'] or not self.is_free(critic_node,self.neighbors): #means that the agent having priority will move to an occupied node
                     is_free =False # the next_next_node is not free

               if n['pos']==critic_node:
                   is_critic_node_free=False
                   id_critic=n['AgentID']
                   prohibited_node = agent_having_priority['pos']

            if is_free :
                   solution[agent_having_priority['AgentID']] = "move"
                   for n in self.neighbors:
                      if agent_having_priority['AgentID'] != n['AgentID']:
                          solution[n['AgentID']] = "wait"

            elif self.is_free(critic_node,self.neighbors):# not Free, so the agent in this node should go to a free node and let the priority to the agent
                for n in self.neighbors:
                     if agent_having_priority['next_next_node'] == n['pos']:# the agent accupied this node

                         got_free_node = map.get_Up_Down_free_node(n['pos'], critic_node,critic_node)
                         #got_free_node1 = map.get_Up_Down_free_node(agent_having_priority['pos'], critic_node,critic_node)
                         if got_free_node is not None:
                             solution[n['AgentID']] = "move_to_node_and_wait"
                             solution[agent_having_priority['AgentID']] = "move"  # "wait"

                             if n['AgentID'] == self.id:  # it is me
                                 self.my_move_away_node = got_free_node

                             if self.neighbors[0]['AgentID'] == self.id:
                                 logging.info(f'agent{agent_having_priority["AgentID"]} is moving')
                                 logging.info(f'agent{n["AgentID"]} is moving_and_waiting in {got_free_node} ')

                         else:

                           if map.is_free(critic_node):
                              got_free_node1 = map.free_neighboring_node(critic_node,self.neighbors)
                              solution[n['AgentID']] = "move_to_node_via_critic_node"
                              if n['AgentID'] == self.id:  # it is me
                                 self.my_move_away_node = got_free_node1
                              solution[agent_having_priority['AgentID']] = "wait"
                              if agent_having_priority['AgentID'] == self.id:  # it is me
                                 self.got_priority_last_step = True
                              if self.neighbors[0]['AgentID'] == self.id:
                                 logging.info(f'agent{n["AgentID"]} is moving_via_critic_node_and_waiting in {got_free_node1}')

                     else:  # wait and let the other agent pass
                         solution[n['AgentID']] = "wait"

            else:  # the agent in the critic node should move out away
              got_free_node=None
              solved = False
              if self.neighbors[0]['im_done'] or self.neighbors[1]['im_done']or self.neighbors[2]['im_done']:


                  for agent in self.neighbors:
                      if agent['im_done']:  # self.im_done :#and self.action=="wait" : # :
                          # if not is_critic_node_free : #critic node is not free
                          # if id_critic ==self.id: #i need to move out away
                          if agent['AgentID'] == self.id:  #
                              self.im_done = False
                              self.is_last_node = False

                              solution[agent['AgentID']] = "move"

                              #self.moving_away = True
                              got_to_node= map.free_neighboring_node(self.position,self.neighbors)
                              logging.info(f'######################################## --> returned node: {got_to_node}')
                              if got_to_node is not None :
                                  self.remaining_path[0:0] = [got_to_node]

                                  logging.info(f'agent{self.id} done --> move_away--remaining path: {self.remaining_path}')
                              else:
                                  solution[agent['AgentID']] = "wait"


                      elif agent['AgentID'] == priority_agent:
                           solution[agent['AgentID']] = "move"
                      else:
                          solution[agent['AgentID']] = "wait"

              else:
                for n in self.neighbors:
                    if n['pos'] == critic_node:
                           solution[n['AgentID']] = "move"
                           if n['AgentID']==self.id:
                               self.moving_away=True
                           solved=True
                           for agent in self.neighbors:
                               if agent['AgentID']!= n['AgentID'] :
                                   solution[agent['AgentID']] = "wait"########################################
                           break

        elif len(self.neighbors) == 2:  # if there are only 2 AGVs

          if self.is_free(critic_node,self.neighbors):  # no agent in the critic_node

              for n in self.neighbors:
                   if n['AgentID']!=agent_having_priority['AgentID']:
                       if n['pos']==agent_having_priority['next_next_node']:  # means the other agent located on my node

                                got_free_node = map.get_Up_Down_free_node(n['pos'], critic_node, critic_node)
                                got_free_node1 = map.get_Up_Down_free_node(agent_having_priority['pos'], critic_node, critic_node)

                                if self.neighbors[0]['AgentID'] == self.id:
                                    logging.info(f'Function get_Up_Down_free_node returned of the other agent: {got_free_node}')
                                    logging.info( f'Function get_Up_Down_free_node returned of the priority-agent: {got_free_node1}')

                                if got_free_node is not None and got_free_node != n['pos']:  # and got_free_node != critic_node:
                                    solution[agent_having_priority['AgentID']] = "move"
                                    solution[n['AgentID']] = "move_to_node_and_wait"

                                    if n['AgentID'] == self.id:  # it is me
                                        self.my_move_away_node = got_free_node
                                # change the priority if the other agent has a neighbour free node
                                elif got_free_node1 is not None and got_free_node1 != agent_having_priority['pos'] and got_free_node1 != critic_node :
                                     solution[agent_having_priority['AgentID']] = "move_to_node_and_wait"

                                     if agent_having_priority['AgentID'] == self.id:  # it is me
                                        self.my_move_away_node = got_free_node1

                                     solution[n['AgentID']] = "move"
                                     priority_agent = n['AgentID']

                                else:
                                   got_free_node2 = map.free_neighboring_node(critic_node,self.neighbors)
                                   #got_free_node2, _ = map.get_Up_Down_free_node(critic_node, agent_having_priority['pos'],n['pos'])

                                   if got_free_node2 is not None:
                                       solution[n['AgentID']] = "move_to_node_via_critic_node"
                                       solution[agent_having_priority['AgentID']] = "wait"
                                       if n['AgentID'] == self.id:  # it is me
                                                self.my_move_away_node = got_free_node2
                                   else:  # if None
                                      # got_free_node2,_ = map.get_move_away_node(n['pos'], critic_node)
                                      solution[agent_having_priority['AgentID']] = "move"
                                      if agent_having_priority['AgentID'] == self.id:  # it is me
                                         self.got_priority_last_step = True

                                      solution[n['AgentID']] = "move_out_away_intersection"

                                      prohibited_node = critic_node  # agent_having_priority['next_next_node']

                                      if self.neighbors[0]['AgentID'] == self.id:
                                           logging.info(f'agent{n["AgentID"]} is moving away')

                       else:
                           solution[agent_having_priority['AgentID']] = "move"
                           solution[n['AgentID']] = "wait"

          else:# the agent in the critic node should move out away
             got_free_node = None
             solved=False
             if self.neighbors[0]['im_done'] or self.neighbors[1]['im_done'] :

                 for agent in self.neighbors:
                   if agent['im_done']:  # self.im_done :#and self.action=="wait" : # :
                       # if not is_critic_node_free : #critic node is not free
                       # if id_critic ==self.id: #i need to move out away
                       if agent['AgentID'] == self.id:  #
                           self.im_done = False
                           self.is_last_node = False

                           solution[agent['AgentID']] = "move"
                           # self.action = "move"
                           self.moving_away = True
                           pos = self.neighbors[0]['pos']
                           if pos == self.position:  # it is me
                               # self.move_out_away(map, self.neighbors[1]['pos'],self.neighbors[1]['pos'])
                               got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,self.neighbors[1]['pos'],self.neighbors[1]['next_next_node'])
                               if got_to_node is None:
                                   got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,self.neighbors[1]['pos'],self.neighbors[1]['pos'])
                               self.remaining_path[0:0] = [got_to_node]

                           else:
                               # self.move_out_away(map, self.neighbors[0]['pos'], self.neighbors[0]['pos'])
                               got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,self.neighbors[0]['pos'],self.neighbors[0]['next_next_node'])
                               if got_to_node is None:
                                   got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,self.neighbors[0]['pos'],self.neighbors[0]['pos'])
                               self.remaining_path[0:0] = [got_to_node]

                           logging.info(f'agent{self.id} done 11111111--> move_away--remaining path: {self.remaining_path}')

                   else:
                     solution[agent['AgentID']] = "move"

             else:
               for n in self.neighbors:
                 if n['pos'] == critic_node:
                     #if map.is_free(n['next_next_node']):  # next-next node is free
                         solution[n['AgentID']] = "move"
                         if n['AgentID'] == self.id:
                             self.moving_away = True
                             solved=True
                         for agent in self.neighbors:
                             if agent['AgentID'] != n['AgentID']:
                                     solution[agent['AgentID']] = "wait"

        #for n in self.neighbors:
          #if n['AgentID'] not in list(solution.keys()):
             #solution[n['AgentID']] = "wait"

        self.priority_neighbor = priority_agent

        #if an agent is done, it should move out away


        if self.neighbors[0]['AgentID'] == self.id:
            logging.info(f'solution is :{solution}')


        for agent in self.neighbors:
            if agent['AgentID'] == self.id :
               self.action = solution[self.id]
               break


        if self.action == "move_out_away_intersection":
            self.move_out_away_intersection(map, self.critic_node,self.critic_node)  # self.position

        if self.action == "move_out_away":
            self.move_out_away(map,self.critic_node, self.critic_node)  # self.position

        if self.action == "move_backward":
            self.move_out_away_intersection(map,threshold_node ,threshold_node)

        if self.action == "move_to_node":
            self.move_to_node(map,self.my_move_away_node)  # stay only on time-step in this node

        if self.action == "move_to_AGV_node":
            self.move_to_AGV_node(map,self.my_move_away_node)

        if self.action == "move_to_node_via_critic_node":
            self.move_to_node_via_critic_node(map,self.my_move_away_node)

        if self.action == "move_to_node_and_wait":
           self.move_to_node_and_wait(map,self.my_move_away_node) # move to this node and stay two time-steps there

    def solve_opposite_conflict(self, map, critic_node:tuple)->tuple: ############################################################
        """
        return the id of the agent with the priority to move in the next step
        """
        solution = {}
        candidates = self.neighbors.copy()
        self.critic_node = critic_node
        if self.neighbors[0]['AgentID'] == self.id:
         logging.info(f'-----------< solving an opposite_conflict in nodes :  {critic_node}>--------------------')
        priority_agent = None

        if self.neighbors[0]['AgentID'] == self.id:
            self.conflict_agent=self.neighbors[1]['AgentID']
        else:
            self.conflict_agent = self.neighbors[0]['AgentID']

        if self.neighbors[0]['AgentID'] == self.id:
           logging.info('---Determine which agent will have priority for an opposite_conflict---')
           logging.info(f'candidates : {self.neighbors}')
           logging.info(f'rule 1: choose the agent having the state moving_away=True')
          #rule 0 : is the agent in critic node ?

        #rule 1: choose the agent having the state 'moving_away' True
        if candidates[0]['moving_away'] and candidates[0]['conflict_agent']!=candidates[1]['AgentID'] and candidates[0]['next_node']!=candidates[0]['my_last_node']: #not candidates[0]['moving_backward']:
            #if not candidates[1]['moving_away']:
                priority_agent = candidates[0]['AgentID']
                if candidates[0]['AgentID'] == self.id:
                    self.moving_away = False
                # self.my_move_away_node = None
                if self.neighbors[0]['AgentID'] == self.id:
                    logging.info(f'Agent {priority_agent} is moving away, and thus has the priority')

        elif candidates[1]['moving_away'] and candidates[1]['conflict_agent']!=candidates[0]['AgentID']and candidates[0]['next_node']!=candidates[0]['my_last_node']:#not candidates[1]['moving_backward']:
            #if not candidates[0]['moving_away']:
                priority_agent = candidates[1]['AgentID']
                if candidates[1]['AgentID'] == self.id:
                    self.moving_away = False

                if self.neighbors[1]['AgentID'] == self.id:
                    logging.info(f'Agent {priority_agent} is moving away, and thus has the priority')

        if priority_agent is None:
             #rule 2 : choose the agent having a free neighbour node
             got_free_node1 = map.get_Up_Down_free_node(candidates[0]["pos"], candidates[1]["pos"],candidates[1]["next_next_node"])#############
             got_free_node2 = map.get_Up_Down_free_node(candidates[1]["pos"], candidates[0]["pos"],candidates[0]["next_next_node"])#############

             if self.neighbors[0]['AgentID'] == self.id:
                logging.info(f'rule 1 : failed')
                logging.info(f'rule 2 : choose the agent having a free neighbour node')
                logging.info(f'free node of {candidates[0]["pos"]}= {got_free_node1}, free node of {candidates[1]["pos"]} = {got_free_node2}')


             if got_free_node1 is None and got_free_node2 is not None: # give the priority to this agent (it hasn't any free neighboring_node to got to)
                     priority_agent = candidates[0]['AgentID']
                     if self.neighbors[0]['AgentID'] == self.id:
                         logging.info(f'Node {got_free_node2} is free : {map.is_free(got_free_node2)}')
                         logging.info(f'Agent {priority_agent} has the priority')

             elif got_free_node2 is None and got_free_node1 is not None:
                     priority_agent = candidates[1]['AgentID']
                     if self.neighbors[0]['AgentID'] == self.id:
                         logging.info(f'Node {got_free_node1} is free : {map.is_free(got_free_node1)}')
                         logging.info(f'Agent {priority_agent} has the priority')

             elif priority_agent is None and len(candidates) == 2 : # both AGVs have a free neighboring_node or both AGVs don't have any free neighboring_node
                if self.neighbors[0]['AgentID'] == self.id:
                   logging.info(f'rules 2 failed')
                #if an agent is moving backward, then it should continue and give the priority to the other agent
                for agent in candidates:
                  if agent['moving_backward']:# the other agent will have the priority because it had the last step
                     if self.neighbors[0]['AgentID'] == self.id:
                        logging.info(f'the agent{agent["AgentID"]} is moving backward,thus the other agent will have the priority')
                     if agent['AgentID'] == self.id:
                        self.moving_backward = False
                     candidates.remove(agent)
                     priority_agent = candidates[0]['AgentID']
                     if self.neighbors[0]['AgentID'] == self.id:
                         logging.info(f'Agent {priority_agent} has the priority')

                     break

                if priority_agent == None and len(candidates) > 1:
                  logging.info(f'rule 03 : choose the agent with highest num_pos_requests')
                  # rule 03 : choose the agent with highest num_pos_requests
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
                    if self.neighbors[0]['AgentID'] == self.id:
                      logging.info(f'rule 03: the {candidates[0]["pos"]} has highest num_pos_requests and thus has the priority')

                if priority_agent ==None and len(candidates) > 1: # in case of equality
                    if self.neighbors[0]['AgentID'] == self.id:
                      logging.info(f'rules 3 failed')
                      logging.info(f'rule 04 : choose the agent with highest remaining_nodes')
                    remaining_node=[]
                    for i in candidates:
                        remaining_node.append(i['remaining_nodes'])

                    indx = remaining_node.index(max(remaining_node))
                    longest_path = remaining_node[indx]
                    newcandidates = [i for i in candidates if i['remaining_nodes'] == longest_path]
                    candidates.clear()
                    candidates.extend(newcandidates)

                    if len(candidates) == 1:  #if only one candidate left then it has the priority
                        priority_agent = candidates[0]['AgentID']
                        if self.neighbors[0]['AgentID'] == self.id:
                          logging.info(f'priority rule 04 {candidates[0]["pos"]} has longest path and thus has the priority')

                    # rule 04: in case every other rule failed , choose the agent with the highest id
                    else:
                        if self.neighbors[0]['AgentID'] == self.id:
                          logging.info(f'rule 04 failed, we take the agent with the highest id')
                        priority_agent = max([agent['AgentID'] for agent in candidates])

        self.priority_neighbor = priority_agent

       #update variables:
        self.moving_away     = False
        self.moving_backward = False
        self.got_priority_last_step = False  #


        for neighbor in self.neighbors: # two agents
            if neighbor['AgentID'] == priority_agent:
               self.critic_node = neighbor['pos'] # this is the critic node and priority_agent should move
               if self.neighbors[0]['AgentID']==self.id:
                   logging.info(f'critic node : {self.critic_node}')
               break

        prohibited_node=None
        threshould=None

        for neighbor in self.neighbors: # two agents
            if neighbor['AgentID'] == priority_agent:
                solution[neighbor['AgentID']] = "move"
                prohibited_node = neighbor['next_next_node']
                threshould = neighbor['pos']
                if neighbor['AgentID']==self.id:
                   self.action= "move"

        for n in self.neighbors:  # two agents
            if n['AgentID'] != priority_agent:
                #if neighbor['next_node'] == n['pos']:# true: because there are only two agents
                solution[n['AgentID']] = "move_out_away"
                #logging.info(f'######## move_out_away in opposite-conflict: works')
                if n['AgentID'] == self.id:
                   self.action= "move_out_away"
                   self.move_out_away(map, threshould, prohibited_node)

                if self.neighbors[0]['AgentID'] == self.id:
                   logging.info(f'agent{neighbor["AgentID"]} is moving away to free node')

        if self.neighbors[0]['AgentID'] == self.id:
           logging.info(f'calculated solution :{solution}')

       # for agent in self.neighbors:
            #if agent['AgentID'] == self.id:
                #self.action = solution[self.id]

        if self.action =="move":
            # to informs other agents not figured in the list that my node is free to move to
            self.my_pos_will_be_free = True

        if self.action == "move_out_away2":
            logging.info(f'######## move_out_away in opposite-conflict: works')
            self.move_out_away(map, threshould, prohibited_node)
            #self.moving_backward = True

    def solve_intersection_conflict2(self, map, critic_node: tuple) -> tuple:
        """
         return the id of the agent with the priority to move in the next step
        """
        solution = {}
        is_critic_node_free = True
        id_critic = None
        self.critic_node = critic_node
        candidates = self.neighbors.copy()

        if self.neighbors[0]['AgentID'] == self.id:
            logging.info(f'-----------<solving an intersection_conflict in node {critic_node} >--------------------')
        priority_agent = None

        if self.neighbors[0]['AgentID'] == self.id:
            logging.info(f'intersection_conflict candidates:{candidates}')
            logging.info(f'---Determine which agent will have priority for the intersection_conflict---')

        for agent in candidates:
            if agent["got_priority_last_step"]:  # or agent["moving_away"]:
                if self.neighbors[0]['AgentID'] == self.id:
                    logging.info(
                        f'agent{agent["AgentID"]} got priority last step and it will have the priority this step')
                priority_agent = agent['AgentID']
                if priority_agent == self.id:  # it is me
                    self.got_priority_last_step = False  #
                break

        if priority_agent is None and len(candidates) > 1:

            if self.neighbors[0]['AgentID'] == self.id:
                logging.info(f'rules one checking .... neighbors list length = {len(self.neighbors)}')
                logging.info(f'<rule 01 : the next-next node of an agent is free ? > : under checking...')

            if priority_agent is None and len(candidates) > 1:

                # rule 01: is the agent next-next node is free ?
                newlist = []
                for agent in candidates:
                    # if agent['next_next_node'] not in [cand['next_node'] for cand in candidates]:
                    if agent['next_next_node'] not in [candidate['pos'] for candidate in candidates]:
                        newlist.append(agent)
                if len(newlist) > 0:
                    candidates.clear()
                    candidates.extend(newlist)

                if len(candidates) == 1:  # if only one candidate left then it will have the priority
                    priority_agent = candidates[0]['AgentID']
                    if self.neighbors[0]['AgentID'] == self.id:
                        logging.info(f'rule 01: True for agent{priority_agent}')

                # rule 02: choose the agent with successors
            if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02
                if self.neighbors[0]['AgentID'] == self.id:
                    logging.info(f'<rules 2: choose the agent successor agents > : under checking... ')
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

                    if self.neighbors[0]['AgentID'] == self.id:
                        logging.info(f'priority rule 02: True for agent{priority_agent}')

            # rule 03: choose the agent with the most position requests
            if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

                if self.neighbors[0]['AgentID'] == self.id:
                    logging.info(f'rule 01 : failed')
                    logging.info(f'<rules 2: choose the agent with the most position requests> : under checking... ')
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

                    if self.neighbors[0]['AgentID'] == self.id:
                        logging.info(f'priority rule 02: True for agent{priority_agent}')

            # rule 04 : choose the agent with the longest path
            if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02
                if self.neighbors[0]['AgentID'] == self.id:
                    logging.info('rule 02 failed')
                    logging.info(f'< rules 3: choose the agent with the longest path > : under checking... ')
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
                    if self.neighbors[0]['AgentID'] == self.id:
                        logging.info(f'priority rule 03: True for agent{priority_agent}')


                # rule 05: in case every other rule failed , choose the agent with the highest id
                else:
                    priority_agent = max([agent['AgentID'] for agent in candidates])
                    if self.neighbors[0]['AgentID'] == self.id:
                        logging.info(f'rule 03 failed--we take the agent with the highest id : (agent{priority_agent})')

        prohibited_node = None
        threshold_node = None
        # update variables:
        self.moving_away = False
        self.moving_backward = False
        self.got_priority_last_step = False  #
        agent_having_priority = None
        for agent in self.neighbors:
            if agent['AgentID'] == priority_agent:
                agent_having_priority = agent
                break

        if len(self.neighbors) == 4:  # 4 AGVs in an intersection
            moveAGVnode = None
            for n in self.neighbors:
                if n['pos'] == agent_having_priority['next_next_node']:  # and n['AgentID']!= agent_having_priority['AgentID']

                    # get a free node without moving back
                    got_free_node1 = map.get_Up_Down_free_node(n['pos'], critic_node, critic_node)

                    if self.neighbors[0]['AgentID'] == self.id:
                        logging.info(f'get_Up_Down_free_node return--{got_free_node1}')

                    if got_free_node1 is None:
                        solution[n['AgentID']] = "move_to_AGV_node"
                        solution[agent_having_priority['AgentID']] = "wait"

                        # chose the node of the agent to move to
                        condidate_ = self.neighbors.copy()
                        condidate_.remove(n)  # remove my self from the neighbors
                        condidate_.remove(agent_having_priority)  # remove the agent having the priority from the list
                        moveAGVnode, agent_ = self.get_best_AGV_node(map, condidate_, critic_node)

                        #############################################################################################################
                        # TODO: add solution to agent : move_to_node_and_wait
                        got_free_node3, _ = map.get_move_out_away_node(agent_['pos'], critic_node, critic_node)
                        solution[agent_['AgentID']] = "move_to_node_and_wait"
                        if agent_['AgentID'] == self.id:  # it is me
                            self.my_move_away_node = got_free_node3
                        #############################################################################################################

                        if n['AgentID'] == self.id:  # to update the AGV node o move to
                            self.my_move_away_node = moveAGVnode
                        if agent_having_priority['AgentID'] == self.id:
                            self.got_priority_last_step = True

                        if self.neighbors[0]['AgentID'] == self.id:
                            logging.info(f'agent{agent_having_priority["AgentID"]} having priority should wait')
                            logging.info(f'agent{n["AgentID"]} should move to AGV node {moveAGVnode}')


                    else:  # if (got_free_node1 != msg['next_node'] for msg in map.msg_box.values() ):
                        solution[agent_having_priority['AgentID']] = "move"
                        solution[n['AgentID']] = "move_to_node_and_wait"
                        if n['AgentID'] == self.id:  # it is me
                            self.my_move_away_node = got_free_node1

                else:
                    # solution[agent_having_priority['AgentID']] = "move"
                    solution[n['AgentID']] = "wait"
                    if n['AgentID'] == self.id:
                        self.action == "wait"

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
                for n in self.neighbors:
                    if agent_having_priority['next_next_node'] == n['pos']:  # the agent accupied this node

                        got_free_node = map.get_Up_Down_free_node(n['pos'], critic_node, critic_node)
                        # got_free_node1 = map.get_Up_Down_free_node(agent_having_priority['pos'], critic_node,critic_node)
                        if got_free_node is not None:
                            solution[n['AgentID']] = "move_to_node_and_wait"
                            solution[agent_having_priority['AgentID']] = "move"  # "wait"

                            if n['AgentID'] == self.id:  # it is me
                                self.my_move_away_node = got_free_node

                            if self.neighbors[0]['AgentID'] == self.id:
                                logging.info(f'agent{agent_having_priority["AgentID"]} is moving')
                                logging.info(f'agent{n["AgentID"]} is moving_and_waiting in {got_free_node} ')

                        else:

                            if map.is_free(critic_node):
                                got_free_node1 = map.free_neighboring_node(critic_node, self.neighbors)
                                solution[n['AgentID']] = "move_to_node_via_critic_node"
                                if n['AgentID'] == self.id:  # it is me
                                    self.my_move_away_node = got_free_node1
                                solution[agent_having_priority['AgentID']] = "wait"
                                if agent_having_priority['AgentID'] == self.id:  # it is me
                                    self.got_priority_last_step = True
                                if self.neighbors[0]['AgentID'] == self.id:
                                    logging.info(
                                        f'agent{n["AgentID"]} is moving_via_critic_node_and_waiting in {got_free_node1}')

                    else:  # wait and let the other agent pass
                        solution[n['AgentID']] = "wait"

            else:  # the agent in the critic node should move out away
                got_free_node = None
                solved = False
                for n in self.neighbors:
                    if n['pos'] == critic_node:
                        if map.is_free(n['next_next_node']):  # next-next node is free
                            solution[n['AgentID']] = "move"
                            if n['AgentID'] == self.id:
                                self.moving_away = True
                            solved = True
                            for agent in self.neighbors:
                                if agent['AgentID'] == priority_agent:
                                    solution[agent['AgentID']] = "move"
                                else:
                                    solution[agent['AgentID']] = "wait"
                            break

                if got_free_node is None and not solved:
                    got_free_node = map.free_neighboring_node(critic_node, self.neighbors)

                if got_free_node is not None and not solved:
                    for agent in self.neighbors:
                        if agent['pos'] == critic_node:
                            solution[agent['AgentID']] = "move_to_node"
                            if agent['AgentID'] == self.id:  # it is me
                                self.moving_away = True
                                self.my_move_away_node = got_free_node

                        elif agent['AgentID'] == priority_agent:
                            solution[agent['AgentID']] = "move"
                        else:
                            solution[agent['AgentID']] = "wait"

                else:
                    if not solved:
                        for agent in self.neighbors:
                            if agent['pos'] == critic_node:
                                solution[agent['AgentID']] = "move_backward"
                                prohibited_node = critic_node  # agent_having_priority['next_next_node']
                                if self.neighbors[0]['AgentID'] == self.id:
                                    logging.info(f'agent{n["AgentID"]} is moving out away')
                                break

                        for agent in self.neighbors:
                            if agent['AgentID'] == priority_agent:
                                solution[agent['AgentID']] = "move"
                            else:
                                solution[agent['AgentID']] = "wait"


        elif len(self.neighbors) == 2:  # if there are only 2 AGVs

            if self.is_free(critic_node, self.neighbors):  # no agent in the critic_node

                for n in self.neighbors:
                    if n['AgentID'] != agent_having_priority['AgentID']:
                        if n['pos'] == agent_having_priority[
                            'next_next_node']:  # means the other agent located on my node

                            got_free_node = map.get_Up_Down_free_node(n['pos'], critic_node, critic_node)
                            got_free_node1 = map.get_Up_Down_free_node(agent_having_priority['pos'], critic_node,
                                                                       critic_node)

                            if self.neighbors[0]['AgentID'] == self.id:
                                logging.info(
                                    f'Function get_Up_Down_free_node returned of the other agent: {got_free_node}')
                                logging.info(
                                    f'Function get_Up_Down_free_node returned of the priority-agent: {got_free_node1}')

                            if got_free_node is not None and got_free_node != n[
                                'pos']:  # and got_free_node != critic_node:
                                solution[agent_having_priority['AgentID']] = "move"
                                solution[n['AgentID']] = "move_to_node_and_wait"

                                if n['AgentID'] == self.id:  # it is me
                                    self.my_move_away_node = got_free_node
                            # change the priority if the other agent has a neighbour free node
                            elif got_free_node1 is not None and got_free_node1 != agent_having_priority[
                                'pos'] and got_free_node1 != critic_node:
                                solution[agent_having_priority['AgentID']] = "move_to_node_and_wait"

                                if agent_having_priority['AgentID'] == self.id:  # it is me
                                    self.my_move_away_node = got_free_node1

                                solution[n['AgentID']] = "move"
                                priority_agent = n['AgentID']

                            else:
                                got_free_node2 = map.free_neighboring_node(critic_node, self.neighbors)
                                # got_free_node2, _ = map.get_Up_Down_free_node(critic_node, agent_having_priority['pos'],n['pos'])

                                if got_free_node2 is not None:
                                    solution[n['AgentID']] = "move_to_node_via_critic_node"
                                    solution[agent_having_priority['AgentID']] = "wait"
                                    if n['AgentID'] == self.id:  # it is me
                                        self.my_move_away_node = got_free_node2
                                else:  # if None
                                    # got_free_node2,_ = map.get_move_away_node(n['pos'], critic_node)
                                    solution[agent_having_priority['AgentID']] = "move"
                                    if agent_having_priority['AgentID'] == self.id:  # it is me
                                        self.got_priority_last_step = True

                                    solution[n['AgentID']] = "move_out_away_intersection"

                                    prohibited_node = critic_node  # agent_having_priority['next_next_node']

                                    if self.neighbors[0]['AgentID'] == self.id:
                                        logging.info(f'agent{n["AgentID"]} is moving away')

                        else:
                            solution[agent_having_priority['AgentID']] = "move"
                            solution[n['AgentID']] = "wait"

            else:  # the agent in the critic node should move out away
                got_free_node = None
                solved = False
                # if agent['im_done']

                for n in self.neighbors:
                    if n['pos'] == critic_node:

                        if map.is_free(n['next_next_node']):  # next-next node is free
                            solution[n['AgentID']] = "move"
                            if n['AgentID'] == self.id:
                                self.moving_away = True

                                solved = True
                            for agent in self.neighbors:
                                if agent['pos'] != critic_node:
                                    solution[agent['AgentID']] = "move"

                if got_free_node is None and not solved:
                    got_free_node = map.free_neighboring_node(critic_node, self.neighbors)

                if got_free_node is not None and not solved:
                    for agent in self.neighbors:
                        if agent['pos'] == critic_node:
                            solution[agent['AgentID']] = "move_to_node"
                            if agent['AgentID'] == self.id:  # it is me
                                self.my_move_away_node = got_free_node

                        else:
                            solution[agent['AgentID']] = "move"

                else:
                    if not solved:
                        for agent in self.neighbors:
                            if agent['pos'] == critic_node:
                                solution[agent['AgentID']] = "move_backward"
                                prohibited_node = critic_node  # agent_having_priority['next_next_node']
                                if self.neighbors[0]['AgentID'] == self.id:
                                    logging.info(f'agent{n["AgentID"]} is moving out away')
                                break

                        for agent in self.neighbors:
                            if agent['pos'] != critic_node:
                                solution[agent['AgentID']] = "move"
                                threshold_node = agent['pos']
                                break

        # for n in self.neighbors:
        # if n['AgentID'] not in list(solution.keys()):
        # solution[n['AgentID']] = "wait"

        self.priority_neighbor = priority_agent

        # if an agent is done, it should move out away
        for agent in self.neighbors:
            if agent['im_done']:  # self.im_done :#and self.action=="wait" : # :
                # if not is_critic_node_free : #critic node is not free
                # if id_critic ==self.id: #i need to move out away
                if agent['AgentID'] == self.id:  #
                    self.im_done = False
                    self.is_last_node = False

                    solution[agent['AgentID']] = "move"
                    # self.action = "move"
                    self.moving_away = True
                    pos = self.neighbors[0]['pos']

                    if pos == self.position:  # it is me
                        # self.move_out_away(map, self.neighbors[1]['pos'],self.neighbors[1]['pos'])
                        got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,
                                                                                       self.neighbors[1]['pos'],
                                                                                       self.neighbors[1][
                                                                                           'next_next_node'])
                        if got_to_node is None:
                            ######################
                            # got_free_node1 = map.get_Up_Down_free_node(self.neighbors[1]['pos'], self.position, self.position)

                            got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,
                                                                                           self.neighbors[1]['pos'],
                                                                                           self.neighbors[1]['pos'])

                        self.remaining_path[0:0] = [got_to_node]

                    else:
                        # self.move_out_away(map, self.neighbors[0]['pos'], self.neighbors[0]['pos'])
                        got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,
                                                                                       self.neighbors[0]['pos'],
                                                                                       self.neighbors[0][
                                                                                           'next_next_node'])
                        if got_to_node is None:
                            got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,
                                                                                           self.neighbors[0]['pos'],
                                                                                           self.neighbors[0]['pos'])
                        self.remaining_path[0:0] = [got_to_node]

                    logging.info(f'agent{self.id} done --> move_away--remaining path: {self.remaining_path}')
                    self.moving_backward = True

        if self.neighbors[0]['AgentID'] == self.id:
            logging.info(f'solution is :{solution}')

        for agent in self.neighbors:
            if agent['AgentID'] == self.id:
                self.action = solution[self.id]
                break

        if self.action == "move_out_away_intersection":
            self.move_out_away_intersection(map, self.critic_node, self.critic_node)  # self.position

        if self.action == "move_out_away":
            self.move_out_away(map, self.critic_node, self.critic_node)  # self.position

        if self.action == "move_backward":
            self.move_out_away_intersection(map, threshold_node, threshold_node)

        if self.action == "move_to_node":
            self.move_to_node(map, self.my_move_away_node)  # stay only on time-step in this node

        if self.action == "move_to_AGV_node":
            self.move_to_AGV_node(map, self.my_move_away_node)

        if self.action == "move_to_node_via_critic_node":
            self.move_to_node_via_critic_node(map, self.my_move_away_node)

        if self.action == "move_to_node_and_wait":
            self.move_to_node_and_wait(map, self.my_move_away_node)  # move to this node and stay two time-steps there

    def post_negotiation(self, map):

       candidates=[]
       candidates_copy=[]

       for agent1 in map.msg_box.values():
          if agent1['AgentID'] == self.id: 
            #get the agent planned the same next_node with me and negotiate the priority again
            for agent2 in map.msg_box.values() :

              if agent2['AgentID'] != self.id :
                 if agent1["next_node"] != None and agent2["next_node"] != None:
                    if (agent1["next_node"] == agent2["next_node"]) :
                        self.had_conflict = True  # my action will depend only on the conflict_resolution process
                        candidates.append(agent1)#me
                        candidates.append(agent2)# the other agent
                        break


       candidates_copy.extend(candidates)
       both_agents_move = True
       if len(candidates) == 2:
           for agent in candidates:
               if agent["planned_action"]=="wait":
                   both_agents_move=False
                   break

       #negotiation process to determine which agent will have priority :
       priority_agent = None
       if len(candidates)==2 and both_agents_move:
        if candidates[0]['AgentID'] == self.id:
            logging.info(f'--------<post negotiation: agent{candidates[0]["AgentID"]}--> {candidates[0]["next_node"]} and agent{candidates[1]["AgentID"]}--> {candidates[1]["next_node"]} >--------------------')
            logging.info(f'candidates:{candidates}')

        if candidates[0]['moving_away'] and not candidates[1]['moving_away']:
           priority_agent = candidates[0]['AgentID']
           if candidates[0]['AgentID'] == self.id:
              self.moving_away = False
           # self.my_move_away_node = None
           if candidates[0]['AgentID'] == self.id:
               logging.info(f'Agent {priority_agent} is moving away, and thus has the priority')

        if candidates[1]['moving_away'] and not candidates[0]['moving_away']:
           priority_agent = candidates[1]['AgentID']
           if candidates[1]['AgentID'] == self.id:
              self.moving_away = False
           # self.my_move_away_node = None
           if candidates[0]['AgentID'] == self.id:
               logging.info(f'Agent {priority_agent} is moving away, and thus has the priority')


        if priority_agent is None and len(candidates) > 1:
                # rule 00: is the agent next-next node is free ?
                newlist = []
                for agent in candidates:

                  if agent['next_next_node'] not in [candidate['next_node'] for candidate in candidates]:
                    if agent['next_next_node'] not in [candidate['pos'] for candidate in candidates]:
                        newlist.append(agent)

                if len(newlist) > 0:
                    candidates.clear()
                    candidates.extend(newlist)

                if len(candidates) == 1:  # if only one candidate left then it will have the priority
                    priority_agent = candidates[0]['AgentID']

        # rule 01: choose the agent with most successors
        if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

            highest_requests_num = []
            done = False
            for i in candidates:
                highest_requests_num.append(i['successors'])
                if i["im_done"]:
                    done = True
                    break

            if not done:  # if an agent is done, dont applied this rule
                indx = highest_requests_num.index(max(highest_requests_num))
                highest_requests = highest_requests_num[indx]
                newcandidates = [i for i in candidates if i['successors'] == highest_requests]
                candidates.clear()
                candidates.extend(newcandidates)

            if len(candidates) == 1:  # if only one candidate left then it will have the priority
                priority_agent = candidates[0]['AgentID']

        # rule 02: choose the agent with the most position requests
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

        # rule 03 : choose the agent with the longest path
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

            # rule 04: in case every other rule failed , choose the agent with the highest id
            else:
                priority_agent = max([agent['AgentID'] for agent in candidates])

        if candidates[0]['AgentID'] == self.id:
           logging.info(f'End_post_negotiation agent{candidates_copy[0]["AgentID"]} and agent{candidates_copy[1]["AgentID"]}: agent{priority_agent} has the priority')

        for agent in candidates_copy:
          if agent["AgentID"] == self.id: #i am in the condidate
             if priority_agent != self.id :
                self.action = "wait"
                logging.info(f'agent{self.id} changed his action to "wait"')
                break

    def post_negotiation2(self, map):

       candidates=[]
       candidates_copy=[]

       for agent1 in map.msg_box.values():
          if agent1['AgentID'] == self.id: #this is me
            #get the agent planned the same next_node with me and negotiate the priority again
            for agent2 in map.msg_box.values() :

              if agent2['AgentID'] != self.id :
                 if agent1["next_node"] != None and agent2["next_node"] != None:
                    if (agent1["next_node"] == agent2["next_node"]) :
                        self.had_conflict = True  # my action will depend only on the conflict_resolution process
                        candidates.append(agent1)#me
                        candidates.append(agent2)# the other agent
                        break


       candidates_copy.extend(candidates)
       both_agents_move = True
       if len(candidates) == 2:
           for agent in candidates:
               if agent["planned_action"]=="wait":
                   both_agents_move=False
                   break

       #negotiation process to determine which agent will have priority :
       priority_agent = None
       if len(candidates)==2 and both_agents_move:
        if candidates[0]['AgentID'] == self.id:
            logging.info(f'--------<post negotiation: agent{candidates[0]["AgentID"]}--> {candidates[0]["next_node"]} and agent{candidates[1]["AgentID"]}--> {candidates[1]["next_node"]} >--------------------')
            logging.info(f'candidates:{candidates}')

        if candidates[0]['moving_away'] and not candidates[1]['moving_away']:
           priority_agent = candidates[0]['AgentID']
           if candidates[0]['AgentID'] == self.id:
              self.moving_away = False
           # self.my_move_away_node = None
           if candidates[0]['AgentID'] == self.id:
               logging.info(f'Agent {priority_agent} is moving away, and thus has the priority')

        if candidates[1]['moving_away'] and not candidates[0]['moving_away']:
           priority_agent = candidates[1]['AgentID']
           if candidates[1]['AgentID'] == self.id:
              self.moving_away = False
           # self.my_move_away_node = None
           if candidates[0]['AgentID'] == self.id:
               logging.info(f'Agent {priority_agent} is moving away, and thus has the priority')


        if priority_agent is None and len(candidates) > 1:
                # rule 00: is the agent next-next node is free ?
                newlist = []
                for agent in candidates:

                  if agent['next_next_node'] not in [candidate['next_node'] for candidate in candidates]:
                    if agent['next_next_node'] not in [candidate['pos'] for candidate in candidates]:
                        newlist.append(agent)

                if len(newlist) > 0:
                    candidates.clear()
                    candidates.extend(newlist)

                if len(candidates) == 1:  # if only one candidate left then it will have the priority
                    priority_agent = candidates[0]['AgentID']

        # rule 01: choose the agent with most successors
        if priority_agent is None and len(candidates) > 1:  # in case of equality in rule 01 apply rule 02

            highest_requests_num = []
            done = False
            for i in candidates:
                highest_requests_num.append(i['successors'])
                if i["im_done"]:
                    done = True
                    break

            if not done:  # if an agent is done, dont applied this rule
                indx = highest_requests_num.index(max(highest_requests_num))
                highest_requests = highest_requests_num[indx]
                newcandidates = [i for i in candidates if i['successors'] == highest_requests]
                candidates.clear()
                candidates.extend(newcandidates)

            if len(candidates) == 1:  # if only one candidate left then it will have the priority
                priority_agent = candidates[0]['AgentID']

        # rule 02: choose the agent with the most position requests
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

        # rule 03 : choose the agent with the longest path
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

            # rule 04: in case every other rule failed , choose the agent with the highest id
            else:
                priority_agent = max([agent['AgentID'] for agent in candidates])

        if candidates[0]['AgentID'] == self.id:
           logging.info(f'End_post_negotiation agent{candidates_copy[0]["AgentID"]} and agent{candidates_copy[1]["AgentID"]}: agent{priority_agent} has the priority')

        for agent in candidates_copy:
          if agent["AgentID"] == self.id: #i am in the condidate
             if priority_agent != self.id :
                self.action = "wait"
                logging.info(f'agent{self.id} changed his action to "wait"')
                break

    def post_coordination_to_solve_conflict(self, map) -> tuple:
        """
        post negotiation to coordinate the planned action
        """
        i=0
        # only the agent did not participate in the conflict resolution should execute this code
        if not self.had_conflict and i== 0:

                my_predecessors = self.get_predecessors(map)

                if my_predecessors is not None and len(my_predecessors) > 0:

                    leader=None

                    for i in range(len(my_predecessors)-1):
                        if self.is_agent_implied_in_opposite_conflict(map,my_predecessors[i]) or self.is_agent_implied_in_intersection_conflict(map,my_predecessors[i]):
                          leader = my_predecessors[i] # i will follow the cation of my predecessors
                          break

                    if leader is None:
                    #else:
                       leader = my_predecessors[-1]

                    #leader_planned_action = my_predecessors[-1]["planned_action"]

                    leader_planned_action =leader["planned_action"]

                    if leader_planned_action == "wait":
                        self.action = "wait"
                        logging.info(f'--------<post coordination>----------')
                        logging.info(f'agent{self.id}: my_predecessors : {my_predecessors}')
                        logging.info(f'leader {leader}')
                        logging.info(f'leader_planned_action {leader["AgentID"]} : {leader_planned_action}')
                        logging.info(f'action of agent {self.id} changed to --wait-- ')

                    else:  # leader will move

                        #TODO: Add If im_done

                        #check the next-pos of the leader in its remaining_path[]  or just check if it is moving backward
                        if leader["next_node"] ==leader["my_last_node"]:#["moving_backward"]:
                            #TODO : false you can't test if it movingbackward like this

                            self.action = "move_away" #
                            predecessor = self.get_predecessor_agent(map, self.id)
                            got_to_node, self.moving_backward = map.get_move_out_away_node(self.position,predecessor['pos'],predecessor['pos'])
                            self.remaining_path[0:0] = [got_to_node,self.position]

                            logging.info(f'--------<post coordination>----------')
                            logging.info(f'agent{self.id}: my_predecessors : {my_predecessors}')
                            logging.info(f'leader {leader}')
                            logging.info(f'leader_planned_action {leader["AgentID"]} : {leader_planned_action}')
                            logging.info(f'action of agent {self.id} changed to --moving_backward--to {got_to_node} ')
                        #else:
                            #self.action = "move"

    def move_out_away(self, map, threshold_node,prohibited_node):
        """
                 used when there are more than one critic node
                """
        got_to_node, self.moving_backward = map.get_move_out_away_node(self.position, threshold_node,prohibited_node)
        logging.info(f'The returned free node for the agent{self.id}: {got_to_node}')

        if got_to_node is None:
           got_to_node, self.moving_backward = map.get_move_out_away_node(self.position, threshold_node,threshold_node)

        map.new_paths_node[self.id] = got_to_node
        self.im_done = False

        if type(got_to_node) is dict:
            got_to_node = got_to_node['pos']

        if not self.moving_backward:  # got_to_node != self.last_node:
            # TODO:Try to plan another path
            logging.info(f'agent {self.id} will move to a free node and return to its pos: {[got_to_node, self.position]}')
            # add the move away path to the remaining path
            self.moving_away = True
        else:
            self.moving_away = False
        # update the path:
        self.remaining_path[0:0] = [got_to_node, self.position]
        logging.info(f'move_away--remaining path: {self.remaining_path}')
        self.my_move_away_node = got_to_node

    def move_out_away_intersection(self, map, threshold_node, prohibited_node):
        """
           used when there are more than one critic node
        """
        got_to_node, self.moving_backward = map.get_move_out_away_node(self.position, threshold_node, prohibited_node)
        logging.info(f'move_out_away_intersection for the agent{self.id}: {got_to_node}')

        map.new_paths_node[self.id] = got_to_node

        self.im_done = False

        if not self.moving_backward:  # got_to_node != self.last_node:

            self.moving_away = True

            self.remaining_path[0:0] = [got_to_node, got_to_node , self.position]
            logging.info(f'--new path: {self.remaining_path}')

        else:
           self.moving_away = True
           self.remaining_path[0:0] = [got_to_node, self.position]
           logging.info(f'--new path: {self.remaining_path}')

        self.my_move_away_node = got_to_node

    def move_away(self, map, threshold_node):
        """
         used when there are more than one critic node
        """
        got_to_node, self.moving_backward = map.get_move_away_node(self.position, threshold_node)
        logging.info(f'The returned free node for the agent{self.id}: {got_to_node}')
        self.im_done = False
        map.new_paths_node[self.id]=got_to_node

        if type(got_to_node) is dict:
            got_to_node = got_to_node['pos']

        if not self.moving_backward :#got_to_node != self.last_node:
            # TODO:Try to plan another path
            logging.info( f'agent {self.id} will move to a free node and return to its pos: {[got_to_node, self.position]}')
            # add the move away path to the remaining path
            self.moving_away = True
        else:
            self.moving_away = False
        #update the path:
        self.remaining_path[0:0] = [got_to_node, self.position]
        logging.info(f'move_away--remaining path: {self.remaining_path}')
        self.my_move_away_node = got_to_node

    def get_best_AGV_node(self,map, candidates,critic_node):
      priority_agent=None
      node  = None
      agent_= None

      # rule 02 : choose the agent with the lest position requests
      if len(candidates) > 1:
        # rule 01:choose the agent with a free neighbouring node
        got_free_node1 = map.get_Up_Down_free_node(candidates[0]["pos"],critic_node,critic_node )#candidates[1]["pos"], candidates[1]["next_next_node"])  #############
        got_free_node2 = map.get_Up_Down_free_node(candidates[1]["pos"],critic_node,critic_node )#candidates[0]["pos"],candidates[0]["next_next_node"])

        if got_free_node1 is None and got_free_node2 is not None:
           candidates.remove(candidates[0])

        elif got_free_node2 is None and got_free_node1 is not None:
            candidates.remove(candidates[1])

        if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']


      #choose the AGV who not located in my nex_next_node
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

      if len(candidates) > 1:
          #rule 01:choose the agent with least num_successors
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

      if len(candidates)>1:
          #rule 02 : choose the agent with least num_pos_requests
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

      # rule 03 : choose the agent with the least path
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


      for agent in candidates:
         if agent['AgentID'] == priority_agent :
              node=agent['pos']
              agent_=agent
              break

      return node,agent_

    def move_backward(self,map):
        logging.info(f'agent {self.id} will move backward ')
        self.remaining_path.insert(0,self.last_node)
        map.new_paths_node[self.id] = self.last_node
        logging.info(f'move_backward--remaining path: {self.remaining_path}')
        self.my_move_away_node=self.last_node
        self.moving_away = False
        self.moving_backward = True

    def move_to_node_and_wait(self,map, node_):

        if node_ is not None:
          map.new_paths_node[self.id] = node_
          self.remaining_path[0:0] = [node_, node_,self.position]#
          logging.info(f'Agent in {self.position} is moving_and_waiting--remaining path: {self.remaining_path}')
          self.my_move_away_node = node_
          self.moving_away = True
          self.im_done = False

        else:
          logging.info(f'-----move_to_node_and_wait--The node is None ')

    def move_to_node(self,map, node_):

        if node_ is not None:
          map.new_paths_node[self.id] = node_
          self.remaining_path[0:0] = [node_, self.position]#
          logging.info(f'Agent in {self.position} is moving_to_node--remaining path: {self.remaining_path}')
          self.my_move_away_node = node_
          self.moving_away = True
          self.im_done = False

        else:
          logging.info(f'-----move_to_node_and_wait--The node is None ')

    def move_to_node_via_critic_node(self,map,my_move_away_node):

        map.new_paths_node[self.id] = my_move_away_node

        self.remaining_path[0:0] = [self.critic_node, my_move_away_node]
        logging.info(f'Agent{self.id}--remaining path: {self.remaining_path}')
        self.my_move_away_node = my_move_away_node
        self.moving_away = True
        self.im_done = False

    def move_to_AGV_node(self,map, got_to_node):
        """
        define the path of the agent required to move away
        """
        logging.info(f'agent {self.id} will move to critic_node, AGV_node, critic_node: {[self.critic_node, got_to_node, self.critic_node]} ')
        #add the move away path to the remaining path

        map.new_paths_node[self.id] = got_to_node
        self.im_done = False
        self.remaining_path[0:0] = [self.critic_node, got_to_node]#, self.critic_node
        logging.info(f'move_to_AGV_node--remaining path: {self.remaining_path}')
        self.moving_away = True

    def next_step(self,map)->None:
        """ Plan the next step of the agent
        :returns: The next node
        """

        # plan the full path if you didn't do that before
        if len(self.target_list) > 0:
            if self.remaining_path is None :
                self.plan_all_path(map)
                # resolve any conflicts before moving the next step

        if self.remaining_path is None or len(self.remaining_path)==0:
            self.im_done = True

            self.target_list = [self.position]
            self.remaining_path = [self.position]
            self.next_waypoint = self.position  # need it to know

        else:
            if len(self.remaining_path)>=1:
                # calculate the next target and next waypoint
                if self._current_target_id < len(self.target_list):
                   self.next_target   = self.target_list[self._current_target_id]
                self.next_waypoint = self.remaining_path[0]



                # if you get to the next target
                if self.next_waypoint == self.next_target:
                    if self._current_target_id + 1 < len(self.target_list):
                        self._current_target_id += 1

        #self.send_my_data(map)

    def move(self, map, sim_time, time_lapsed: float = 0):

        logging.info(f'##############################################')
        # wait for this step
        logging.info(f'Agent {self.id} --->action:{self.action}')


        if len(self.remaining_path) > 0:
            self.next_target   = self.target_list[self._current_target_id]
            self.next_waypoint = self.remaining_path[0]
            self.steps += 1

        if self.action == "wait":
            self.wait()
            logging.info(self.position)

        else: # your planned action is move
           if (self.next_waypoint is None):
               ctypes.windll.user32.MessageBoxW(0, f"Agent{self.id} coming from{self._position} is trying to reach a None node", "Error !", 1)
           else:
                self.face_to(self.next_waypoint)
                logging.info(f'{self._position}-->{self.next_waypoint}')
                self.last_node = self._position
                self.my_pos_will_be_free = True
                self.my_move_away_node=None

                #update the graph
                map.set_as_free(self._position)
                self._position = self.next_waypoint
                map.graph.nodes[self._position]["agent"] = self # map.add_agent_to_map(self)
                map._graph.nodes[self._position]["state"] = 'agent'

                if self.is_last_node: # if this node is the last one in the path, then im_done
                    self.is_last_node = False
                    self.im_done   = True

                #update self.remaining_path by removing the waypoint from it
                if self.next_waypoint in self.remaining_path:
                   self.remaining_path.remove(self.next_waypoint)
                   if len(self.remaining_path) == 0: #
                       self.is_last_node = True



                self.storage_container.append( {'sim_time': round(sim_time, 2), 'steps': self.steps, 'num_targets': len(self.target_list),
                                                 'num_conflicts': self.num_conflicts})

    def plan(self,map)->list:
        """
        method for planning the agent's path to its targets
        :param path_finder: the astar planner
        :return:
        """
        # plan a path to the next target
        target = self.target_list[self._current_target_id]
        self.path = self._path_finder.astar_planner(map.graph, self.position, target)
        map.set_as_path(self.path)

        return self.path

    def plan_all_path(self,map)->None:
        """
        method for planning the agent's path to its targets
        :param path_finder: the astar planner
        :return:
        """
        # the initial waypoints before any plan is made
        logging.info(self.target_list)
        general_waypoints = [self.position] + [target for target in self.target_list ]
        self.remaining_path = []
        #planning the full path including the targets
        self.last_node = self.position
        for i in range(0,len(general_waypoints)-1):
            #calculate obstacle free path between two general waypoints
            path_i = self._path_finder.astar_planner(map.graph, general_waypoints[i], general_waypoints[i+1])

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