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
        self.neighbors = []
        self.priority_neighbor = None
        self.action = "wait"

        # DATA TO BE STORED
        self.steps = 0
        self.num_conflicts = 0
        self.storage_container = [{'sim_time':0,'steps':self.steps,'num_targets':len(self.target_list),'num_conflicts':self.num_conflicts}]


    def send_my_data(self,map:Map):
        """
        send the agent data to the map object
        in order to communicate with other agents
        """
        num_pos_requests,_ = self.pos_requests(self.position)
        #logging.debug(f'agent: {self.id}, requests : {num_pos_requests}')

        data={"AgentID": self.id,
              "pos": (self.row, self.col),
              "remaining_nodes": len(self.remaining_path),
              "next_node": self.remaining_path[0] if len(self.remaining_path)>0 else self.target_list[-1] ,
              "next_next_node": self.remaining_path[1] if len(self.remaining_path)>1 else self.target_list[-1] ,
              "num_pos_requests":num_pos_requests ,
              "moving_away":self.moving_away,
              "moving_backward":self.moving_backward,
              "moving_away_node": self.my_move_away_node,
              "my_last_node":self.last_node,
              "got_priority_last_step":self.got_priority_last_step,
              "my_pos_will_be_free":self.my_pos_will_be_free}
        map.msg_box[self.id] = data

    def pos_requests(self,position:tuple)->tuple:
        """
        Find the agents requesting this position in the next or the next-next step
        args:
        position:(tuple) the coordinates of the node ( row,col)

        return:
         pos_requests:(int) the number of agents requesting this position
         in the next step or in the next-next step
         pos_requests_list: (list) list of agents requesting this position
         in the next step or in the next-next step

        """
        pos_requests = 0
        pos_requests_list = []
        for agent in self.neighbors:
            if ( (agent['next_node'] == position) or (agent['next_next_node'] == position) ) and agent['AgentID'] != self.id:
                pos_requests+=1
                pos_requests_list.append(agent)

        return pos_requests,pos_requests_list

    def handle_conflicts(self,map):
        # solve conflicts for all agents
        is_conflict, is_chase, critic_node = self.check_for_conflict(map)

        if is_conflict:
            if len(critic_node) == 1:
               self.critic_node = critic_node[0]

               #if not map.occupied(self.critic_node):
               _, solution = self.solve_intersection_conflict(map, critic_node)

               logging.debug(f'there is a conflict for robot with id {self.id} node {critic_node}\nthe solution solution was:',solution,self.id)
            else:
               _, solution = self.solve_opposite_conflict(map, critic_node)

        elif is_chase:
            pass

        else:

            self.action = "move"
            self.my_pos_will_be_free = True

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
    def get_chase_neighbors(self, map):
        """
        return: the neighbors list of this agent
        """
        neighbors = []
        my_next_node = None
        if len(self.remaining_path) > 0:
            my_next_node = self.remaining_path[0]

        for msg in map.msg_box.values():
            if msg["next_node"] != None:
                if      (msg["next_node"] == self._position) or \
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
                if msg["next_node"] != None:
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

    def check_for_conflict(self,map):

        if len(self.remaining_path)>=1:
            my_next_node = self.next_waypoint
        else:
            my_next_node = self.target_list[-1]

        next_nodes = []
        for msg in map.msg_box.values():
            if msg["AgentID"] == self.id:
                continue
            else:
                # 1-intersection conflict
                if (msg["next_node"] == my_next_node) :
                    self.neighbors = self.get_intersection_conflict_neighbors(map)
                    next_nodes.append(my_next_node)
                    logging.debug(f'Intersection conflict in node: {my_next_node}')
                    return True,False, next_nodes

                #2-opposite conflict (two conflict nodes)
                elif (msg["next_node"] == self.position) and (my_next_node == msg["pos"]):
                    self.neighbors = self.get_opposite_conflict_neighbors(map)
                    next_nodes.append(self.position)
                    next_nodes.append(my_next_node)

                    logging.debug(f'Opposite conflict nodes: {next_nodes}')# between agent {self.id} and agent {msg["AgentID"]}')
                    return True,False, next_nodes

                # 3- There are other AGVs docked on my path
                elif (msg["next_node"] != self.position) and (my_next_node == msg["pos"] and msg["next_node"]==msg["pos"]):
                     self.neighbors = self.get_opposite_conflict_neighbors(map)
                     next_nodes.append(my_next_node)
                     logging.debug(f'There are other AGVs docked on my path in node: {my_next_node}')
                     return True,False, next_nodes

                # 4- chase-conflict
                #elif msg["next_node"] != self.position and my_next_node == msg["pos"] and msg["next_node"]!=msg["pos"]:
                     #self.neighbors = self.get_chase_neighbors(map)
                     #next_nodes.append(my_next_node)
                     #logging.debug(f'There are other AGVs docked on my path in node: {my_next_node}')
                     #return False, True,next_nodes

        self.neighbors = []
        return False,False, None


    def solve_intersection_conflict(self, map, critic_node:tuple)->tuple:
        """
         return the id of the agent with the priority to move in the next step
        """
        solution = {}
        if self.neighbors[0]['AgentID'] == self.id:
          print('-----------<solving an intersection_conflict>--------------------')
        priority_agent = None
        candidates = self.neighbors.copy()
        if self.neighbors[0]['AgentID'] == self.id:
           logging.debug('conflict candidates msgs: ',candidates)
           logging.info('---Determine which agent will have priority ---')

        for agent in candidates:
            if agent["got_priority_last_step"]:
                priority_agent = agent['AgentID']
                if priority_agent == self.id: # it is me
                   self.got_priority_last_step = False #
                break

        if len(candidates) > 1 and priority_agent is None:

            #rule 01 : is the agent next-next node is free ?
            if self.neighbors[0]['AgentID'] == self.id:
               logging.info(f'rules one checking .... neighbors list length = {len(self.neighbors)}')
            newlist = []
            for agent in candidates:
                if agent['next_next_node'] not in [cand['pos'] for cand in candidates]:
                    newlist.append(agent)
            if len(newlist) >0:
                candidates.clear()
                candidates.extend(newlist)
            if self.neighbors[0]['AgentID'] == self.id:
               logging.info(f'candidate list after applying rule 01 now : {len(candidates)} agents ')

            if len(candidates)==1: # if only one candidate left then it will have the priority
                priority_agent = candidates[0]['AgentID']
                if self.neighbors[0]['AgentID'] == self.id:
                   logging.info('priority rule 01', candidates[0]['pos'])

            # rule 02 : choose the agent with the most position requests
            if len(candidates) > 1 : # in case of equality in rule 01 apply rule 02
                logging.info('rule 01 failed')
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
                       logging.info(f'priority rule 02 {candidates[0]["pos"]}')

            #rule 03 : choose the agent with the longest path
            if len(candidates) > 1: # in case of equality in rule 01 apply rule 02
                    if self.neighbors[0]['AgentID'] == self.id:
                       logging.info('rule 02 failed')
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
                           logging.info(f'priority rule 03: {candidates[0]["pos"]}')


                    #rule 04: in case every other rule failed , choose the agent with the highest id
                    else:
                        if self.neighbors[0]['AgentID'] == self.id:
                           logging.info('rule 03 failed--we take the agent with the highest id')
                        priority_agent = max([agent['AgentID'] for agent in candidates])


        elif priority_agent is None and len(candidates) == 1:
              priority_agent = candidates[0]['AgentID']

        for neighbor in self.neighbors:
            if neighbor['AgentID'] == priority_agent:
                if self.neighbors[0]['AgentID'] == self.id:
                   logging.info(f'agent{neighbor["AgentID"]} has priority to move')
                solution[neighbor['AgentID']] = "move"

                if neighbor['AgentID'] == self.id and not self.got_priority_last_step: # it is me
                   self.got_priority_last_step = True # in the next step, AGV can know that it had the priority last time

                if len(self.neighbors) == 4 : # 4 AGVs in an intersection

                  moveAGVnode = None
                  for n in self.neighbors :
                     if n['AgentID']!=neighbor['AgentID']:
                       # some other agent is in the way
                       if  n['pos'] == neighbor['next_next_node'] :
                          got_free_node1 = map.free_neighboring_node(n['pos'], [n['pos'],n["my_last_node"],critic_node])

                          if got_free_node1 is None :
                              solution[n['AgentID']] = "move_to_AGV_node"
                              solution[neighbor['AgentID']] = "wait"
                              #chose the node of the agent to move to
                              condidate_=self.neighbors.copy()
                              condidate_.remove(n)        # remove my self from the neighbors
                              condidate_.remove(neighbor) # remove the agent having the priority from the list
                              moveAGVnode,agent= self.get_AGV_node(condidate_)

                              #solution[agent['AgentID']] = "move_away" # this agent should move backward ##############################

                              if n['AgentID'] == self.id:  # to update the AGV node o move to
                                 self.my_move_away_node= moveAGVnode

                              if self.neighbors[0]['AgentID'] == self.id:
                                logging.info(f'agent{neighbor["AgentID"]} having priority should wait')
                                logging.info(f'agent{n["AgentID"]} should move away')

                          else:#if (got_free_node1 != msg['next_node'] for msg in map.msg_box.values() ):
                              solution[n['AgentID']] = "move_to_node_and_wait" # move to this free node and wait there

                              if agent['AgentID'] == self.id:  # it is me
                                 self.my_move_away_node = got_free_node1

                elif len(self.neighbors) == 3 :
                  is_free=True
                  for n in self.neighbors:
                    if neighbor['AgentID'] != n['AgentID']:
                       if neighbor['next_next_node'] == n['pos']: # means that the agent having priority will move to an occupied node
                           is_free =False

                  if is_free :
                    solution[neighbor['AgentID']] = "move"
                    for n in self.neighbors:
                       if neighbor['AgentID'] != n['AgentID']:
                          solution[n['AgentID']] = "wait"
                  else: #the agent in this node should go to a free node and let the priority
                   solution[neighbor['AgentID']] = "wait"
                   for n in self.neighbors:
                     if neighbor['next_next_node'] == n['pos']:# the agent accupied this node
                       got_free_node1 = map.free_neighboring_node(n['pos'], [n['pos'], n["my_last_node"]])
                       if got_free_node1 is None:
                           # go to free node
                           pass #########################################################################################
                       else:
                           solution[n['AgentID']] = "move_to_node_and_wait"  # move to this free node and wait there
                           if agent['AgentID'] == self.id:  # it is me
                               self.my_move_away_node = got_free_node1

                elif len(self.neighbors) == 2 : # if there are only 2 AGVs
                  #1-we change the priority and give it to the agent having a free neighbors node
                  got_free_node1 = map.free_neighboring_node(self.neighbors[0]["pos"],[self.neighbors[1]["pos"], self.neighbors[0]["my_last_node"]])
                  got_free_node2 = map.free_neighboring_node(self.neighbors[1]["pos"],[self.neighbors[0]["pos"], self.neighbors[1]["my_last_node"]])
                  if self.neighbors[0]['AgentID'] == self.id:
                    logging.info(f'got_free_node1 = {got_free_node1}, got_free_node2 = {got_free_node2}')

                  if got_free_node1 is None and got_free_node2 is not None:  # give the priority to this agent (it hasn't any free neighboring_node to got to)
                    priority_agent = self.neighbors[0]['AgentID']

                  elif got_free_node2 is None and got_free_node1 is not None:
                    priority_agent = self.neighbors[1]['AgentID']

                  else:  # the two pos have a free neighboring_node, we keep the same defined priority
                    for n in self.neighbors:
                        if neighbor['AgentID'] != n['AgentID']:
                            if neighbor['next_node'] == n['pos']:
                                solution[n['AgentID']] = "move_away"  # the agent should move away

                                if self.neighbors[0]['AgentID'] == self.id:
                                   logging.info(f'agent{n["AgentID"]} is moving away')

            else:  #the other should wait
                if neighbor['AgentID'] not in list(solution.keys()):
                   solution[neighbor['AgentID']] = "wait"
        if self.neighbors[0]['AgentID'] == self.id:
          logging.info('solution is :',solution)
        self.priority_neighbor = priority_agent


        # move away to let priority agent pass
        self.action = solution[self.id]

        if self.action == "move" or self.action == "move_to_AGV_node" or self.action == "move_away" or self.action=="move_to_node_and_wait":
            # to informs other agents not figured in the list that my node is free to move to
            self.my_pos_will_be_free = True

        if self.action == "move_to_AGV_node":
            self.move_to_AGV_node(self.my_move_away_node)

        if self.action == "move_away":
            self.move_away(map, self.critic_node)

        #if self.action == 4:
           #self.move_away(map, [self.critic_node, self.position])# TODO: check again if correct

        if self.action == "move_to_node_and_wait":
           self.move_to_node_and_wait(self.my_move_away_node) # move to this node and stay one step there

        self.send_my_data(map)

        return priority_agent,solution


    def solve_opposite_conflict(self, map, critic_node:tuple)->tuple: ############################################################
        """
        return the id of the agent with the priority to move in the next step
        """
        solution = {}
        if self.neighbors[0]['AgentID'] == self.id:
         print('-----------< solving an opposite_conflict >--------------------')
        priority_agent = None
        candidates = self.neighbors.copy()
        if self.neighbors[0]['AgentID'] == self.id:
          logging.info(f'The candidates : {candidates}')
          logging.info(f'The critic node(s) :  {critic_node}')
          #rule 0 : is the agent in critic node ?
          logging.info('---Determine which agent will have priority---')

        for agent in candidates:
            # rule 1 : choose the agent having the state 'moving_away' True
            if agent['moving_away'] and not agent['moving_backward']:#and (agent['next_node']==critic_node[0] or agent['next_node']==critic_node[1]):
               priority_agent   = agent['AgentID']
               if agent['AgentID']==self.id:
                   self.moving_away = False
               #self.my_move_away_node = None
               if self.neighbors[0]['AgentID'] == self.id:
                  logging.info(f'Agent {priority_agent} is moving away, and thus has the priority')
               break

        if priority_agent is None:
             #rule 2 : choose the agent having a free neighbour node
             #TODO: get the numbbre of free neighboring and give the priority to the agent having smallest number
             got_free_node1 = map.free_neighboring_node(candidates[0]["pos"],[candidates[1]["pos"]])
             got_free_node2 = map.free_neighboring_node(candidates[1]["pos"],[candidates[0]["pos"]])
             if self.neighbors[0]['AgentID'] == self.id:
               logging.info(f'got_free_node1 = {got_free_node1}, got_free_node2 = {got_free_node2}')

             if got_free_node1 is None and got_free_node2 is not None: # give the priority to this agent (it hasn't any free neighboring_node to got to)
                     priority_agent = candidates[0]['AgentID']

             elif got_free_node2 is None and got_free_node1 is not None:
                     priority_agent = candidates[1]['AgentID']

             elif priority_agent is None and len(candidates) ==2 : # both AGVs have a free neighboring_node or both AGVs don't have any free neighboring_node
                if self.neighbors[0]['AgentID'] == self.id:
                   logging.info(f'rules 2 failed')
                #rule02:if an agent is moving backward, then it should continue and give the priority to the other agent
                for agent in candidates:
                  if agent['moving_backward']:# the other agent will have the priority because it had the last step
                     if self.neighbors[0]['AgentID'] == self.id:
                        logging.info(f'the agent{agent["AgentID"]},thus the othere agent will have the priority')
                     if agent['AgentID'] == self.id:
                        self.moving_backward = False
                     candidates.remove(agent)
                     priority_agent = candidates[0]['AgentID']
                     break

                if priority_agent == None and len(candidates) > 1:
                  # rule 03 : choose the agent with the longest path
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
                      logging.info(f'priority rule 03: the {candidates[0]["pos"]} has longest path and thus has the priority')

                if priority_agent ==None and len(candidates) > 1: # in case of equality
                    if self.neighbors[0]['AgentID'] == self.id:
                      logging.info(f'rules 3 failed')
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
                          logging.info(f'priority rule 04 {candidates[0]["pos"]} has many request for his pos and thus has the priority')

                    # rule 04: in case every other rule failed , choose the agent with the highest id
                    else:
                        if self.neighbors[0]['AgentID'] == self.id:
                          logging.info(f'rule 04 failed, we take the agent with the highest id')
                        priority_agent = max([agent['AgentID'] for agent in candidates])

        for neighbor in self.neighbors: # two agents
            if neighbor['AgentID'] == priority_agent:

               self.critic_node = neighbor['pos'] # this is the critic node and priority_agent should move
               if self.neighbors[0]['AgentID']==self.id:
                  logging.info(f'the critic node is {self.critic_node}')
               solution[neighbor['AgentID']] = "move"

               for n in self.neighbors:
                  if neighbor['AgentID']!= n['AgentID']:
                      #if neighbor['next_node'] == n['pos']:# true: because there are only two agents
                      solution[n['AgentID']] = "move_away"
                      if self.neighbors[0]['AgentID'] == self.id:
                         logging.info(f'agent{n["AgentID"]} is moving away to free node')
                      break

        for n in self.neighbors:
          if n['AgentID'] not in list(solution.keys()):
             solution[n['AgentID']] = "wait"

        if self.neighbors[0]['AgentID'] == self.id:
          logging.info(f'calculated solution :{solution}')
        self.priority_neighbor = priority_agent

        # move away to let priority agent pass
        self.action = solution[self.id]

        if self.action =="move":
            # to informs other agents not figured in the list that my node is free to move to
            self.my_pos_will_be_free = True

        if self.action == "move_away":
            self.my_pos_will_be_free = True
            # don't move backward and don't move to the critic_node pos
            self.move_away(map, self.critic_node)#de, self.critic_node])

        self.send_my_data(map)

        return priority_agent,solution


    def move_away(self, map, prohibited_node):
        """
         used when there are more than one critic node
        """
        got_to_node, self.moving_backward = map.get_move_away_node(self.position, prohibited_node)
        logging.info(f'The returned free node : {got_to_node}')

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


    def solve_following_conflict(self, map, critic_node: tuple) -> tuple:
        solution={}
        self.action = solution[self.id]

        if self.action == "move" or self.action == 2 or self.action == "move_away":
            # to informs other agents not figured in the list that my node is free to move to
            self.my_pos_will_be_free = True

        if self.action == 2:
            self.move_to_AGV_node(self.my_move_away_node)

        if self.action == "move_away":
            self.move_away(map, self.critic_node)

        if self.action == 4:
            self.move_away(map, [self.critic_node, self.position])

        self.send_my_data(map)

    def get_AGV_node(self, candidates):
      priority_agent=None
      node  = None
      agent_= None

      # rule 02 : choose the agent with the lest position requests
      if len(candidates) > 1:
        requests_num = candidates[0]['num_pos_requests']
        requests_num = []
        for i in candidates:
            requests_num.append(i['remaining_nodes'])
        indx = requests_num.index(min(requests_num))
        smallestRequ = requests_num[indx]
        newcandidates = [i for i in candidates if i['remaining_nodes'] == smallestRequ]
        candidates.clear()
        candidates.extend(newcandidates)


        if len(candidates) == 1:  # if only one candidate left then it will have the priority
            priority_agent = candidates[0]['AgentID']
            logging.info(f'priority rule 02 {candidates[0]["pos"]}')

        # rule 03 : choose the agent with the least path
        elif len(candidates) > 1:  # in case of equality in rule 01 apply rule 02
            logging.info(f'rule 02 failed')
            remaining_node = []
            for i in candidates:
                remaining_node.append(i['remaining_nodes'])

            indx = remaining_node.index(min(remaining_node))
            longest_path = remaining_node[indx]

            logging.info(remaining_node)
            logging.info(candidates)
            newcandidates = [i for i in candidates if i['remaining_nodes'] == longest_path]
            candidates.clear()
            candidates.extend(newcandidates)

            if len(candidates) == 1:  # if only one candidate left then it will have the priority
                priority_agent = candidates[0]['AgentID']
                logging.info(f'priority rule 03, {candidates[0]["pos"]}')

            # rule 04: in case every other rule failed , choose the agent with the highest id
            else:
                logging.info(f'rule 03 failed let--we take the agent with the highest id')
                priority_agent = max([agent['AgentID'] for agent in candidates])

      for agent in candidates:
         if agent['AgentID'] == priority_agent :
              node=agent['pos']
              agent_=agent
              break

      return node,agent_

    def move_backward(self):
        logging.info(f'agent {self.id} will move backward ')
        self.remaining_path.insert(0,self.last_node)
        logging.info(f'move_backward--remaining path: {self.remaining_path}')
        self.my_move_away_node=self.last_node
        self.moving_away = False
        self.moving_backward = True


    def move_to_node_and_wait(self, node_):
        if node_ is not None:
          map.new_paths_node[self.id] = node_
          self.remaining_path[0:0] = [node_, node_,self.position]
          logging.info(f'move_to_node_and_wait--remaining path: {self.remaining_path}')
          self.my_move_away_node = node_
          self.moving_away = True
        else:
          logging.info(f'move_to_node_and_wait--The node is None ')

    def move_to_AGV_node(self, got_to_node):
        """
        define the path of the agent required to move away
        """
        logging.info(f'agent {self.id} will move to critic_node, AGV_node, critic_node: {[self.critic_node, got_to_node, self.critic_node]} ')
        #add the move away path to the remaining path

        map.new_paths_node[self.id] = got_to_node

        self.remaining_path[0:0] = [self.critic_node, got_to_node, self.critic_node]
        logging.info(f'move_to_AGV_node--remaining path: {self.remaining_path}')
        self.moving_away = True


    def next_step(self,map)->None:
        """ Move the agent
        :env: The env
        :updated_pos: The updated position of agents already moved
        :returns: The new node it would move to
        """
        # plan the full path if you didn't before
        if len(self.target_list) > 0:
            if self.remaining_path is None :
                self.plan_all_path(map)
                # resolve any conflicts before moving the next step

        if self.remaining_path is None:
            self.target_list = [self.position]
            self.remaining_path = [self.position]
            self.next_waypoint = self.position  # need it to know

        else:
            if len(self.remaining_path)>=1:
                # calculate the next target and next waypoint
                self.next_target   = self.target_list[self._current_target_id]
                self.next_waypoint = self.remaining_path[0]

                # if you get to the next target
                if self.next_waypoint == self.next_target:
                    if self._current_target_id + 1 < len(self.target_list):
                        self._current_target_id += 1


        self.send_my_data(map)

    def move(self, map, sim_time, time_lapsed: float = 0):
        # wait for this step
        logging.info(f'Agent {self.id} --->action:{self.action}')
        self.steps += 1

        if len(self.remaining_path) > 0:
            self.next_target   = self.target_list[self._current_target_id]
            self.next_waypoint = self.remaining_path[0]

        if self.action == "wait" and len(self.remaining_path) > 0:
            #check whether an agent will move away to my node
            for msg in map.msg_box.values():
                if msg["pos"]==self.next_waypoint and msg["my_pos_will_be_free"]:
                    self.action == "move"                                               ##############################################
                    logging.info(f'Action of agent {self.id} changed to --move ')
                    break

            if self.action == "move":
                for msg in map.msg_box.values():
                    if msg["next_node"] == self.next_waypoint: # another agent has priority to move to my next node
                        #then, reset my action to 0
                        logging.info(f'Action of agent {self.id} changed to --wait ')
                        self.action == "wait"
                        break

        if self.action == "wait":
            self.wait()
        else: # you can move in this step

                self.face_to(self.next_waypoint)
                self.last_node = (self.row, self.col)
                self.my_pos_will_be_free = False

                #update the graph
                map.graph.nodes[(self.row, self.col)]["agent"] = None
                map.graph.nodes[self._position]["state"] = 'free_space'
                self._position = self.next_waypoint

                map.graph.nodes[self._position]["agent"] = self
                #map.graph.nodes[self._position]["state"] = 'agent'

                # update self.remaining_path by removing the waypoint from it
                if self.next_waypoint in self.remaining_path:
                    self.remaining_path.remove(self.next_waypoint)

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
        logging.info(position)
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


