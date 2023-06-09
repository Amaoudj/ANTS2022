from typing import Callable as Func, List, Tuple
from ..map import Map
from ..agent import agent_placement
from swarmI4.agent.smart_agent import SmartAgent
from swarmI4.agent.agent_placement import custom_placement
import pandas as pd
import os,sys
import csv
import  random
import  logging
import pyautogui


class Swarm(object):
    """ This a wrapper for the all agents """

    def __init__(self,args, agent_generators: List[Tuple[int, Func]], placement_func: Func, my_map: Map):
        """ Create the swarm """
        # create an empty storage folder

        self._agents = []
        self.changed_agents_list = []
        self._positions = {}
        self.Time_Step  = 1
        self.agent_done = 0
        self.create_swarm(args,agent_generators, my_map, placement_func)
        self._my_map = my_map
        self.done    = False
        self.success = True

    # called from RaoufExperiment class
    def create_swarm(self,args, agent_generators: List[Tuple[int, Func]], my_map: Map, placement_func: Func) -> None:
        """ Create the swarm according to the generators """
        self.changed_agents_list.clear()
        self._agents.clear()
        my_map.new_OBS.clear()

        if args.agent_placement == "custom_placement":
            total_number_of_agents = len(my_map.custom_start_list)
        else:
            total_number_of_agents = sum([agent_type[0] for agent_type in agent_generators])

        my_map.neighbors_agents_stat = [None] * total_number_of_agents  # list of the agents stats
        self._my_map = my_map

        for number, gen in agent_generators:
            for i in range(total_number_of_agents):
                position,targets= placement_func(i, total_number_of_agents, my_map)  #

                if args.agent_placement == "random_placement":
                   targets, _ = placement_func(i, total_number_of_agents, my_map)

                agent = gen(position)
                if targets is not None and args.agent_placement != "random_placement":  #
                    agent.target_list.append(targets)
                    my_map.set_as_target(targets)

                agent.id = i

                # set targets as they come in the custom map
                my_map.add_agent_to_map(agent)
                agent.send_my_position(my_map)
                self._agents.append(agent)

    def update_msg_box(self):
        for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.send_my_data(self._my_map)

    def agents_post_coordination(self):
        for i in range(0,6):
          self.update_msg_box()
          for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.post_coordination(self._my_map)
          self.update_msg_box()
          for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.post_negotiation(self._my_map)

        self.update_msg_box()

        for i in range(0,6):
          self.update_msg_box()
          for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.plan_last_step_after_negotiation(self._my_map)

        self.update_msg_box()


    def get_sum_cost(self):
        cost=0
        for agent in self._agents:
            if type(agent) is SmartAgent:
                cost += agent.steps
        return cost


    def move_all(self,simulation_time,dt=0) -> None:
          """
          Move all agents in the swarm
          """

          for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.next_step(self._my_map)

          # update msg box
          self.update_msg_box()

          for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.compute_local_data(self._my_map)

          #print(f'Phase 01 : Handling conflicts ')
          for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.handle_conflicts(self._my_map)

          #print(f'Phase 02 :Agents_post_coordination() ...')
          self.agents_post_coordination()

          self.update_msg_box()

          #print(f'Phase 03 : Agents are moving ...')
          for agent in self._agents:
           if type(agent) is SmartAgent:
             if not agent.im_done:#len(agent.remaining_path) > 0 :
                agent.move(self._my_map,simulation_time, time_lapsed=dt)
                #
           else:
              agent.move(self._my_map, simulation_time, time_lapsed=dt)

          # update msg box
          self.update_msg_box()

          #clear this list for the next use
          self._my_map.new_paths_node.clear()
          self.Time_Step += 1
          self.done = True
          for agent in self._agents:
            if type(agent) is SmartAgent:
                if not agent.im_done:
                    self.done = False
                    break
            else:
                self.done = False
                break
          #logging.info(f'------------< End iteration >-----------------------------')
          for agent1 in self._agents:
            for agent2 in self._agents:
                if agent1.id != agent2.id :
                    if agent1.position == agent2.position :
                       self.success    =  False
                       #pyautogui.alert(text='Agents failed in finding solutions to a deadlock',title='Simulation failed',button='OK')
                       #pyautogui.alert(text='Collision between : ' + str(agent1.id)+' from '+ str(agent1.last_node) + ' and ' +str(agent2.id) +' from ' +str(agent2.last_node), title='Conflict in node'+str(agent1.position), button='OK')
                       #break


    def set_positions(self, position: int) -> None:
        """ Set the same position for all agents in swarm
        :position: The node id
        """
        for agent in self._agents:
            agent.position = position

        self._positions.clear()
        self._positions[str(position)] = len(self._agents)


    @staticmethod
    def store_data(data,folder,file_name):
        # all files of the folder if the folder exists
        df = pd.DataFrame(data)
        fullname = os.path.join(folder, file_name)
        df.to_csv(fullname)

    @property
    def agents(self):
        return self._agents
