from typing import Callable as Func, List, Tuple
from ..map import Map
from ..agent import agent_placement
from swarmI4.agent.smart_agent import SmartAgent
from swarmI4.agent.agent_placement import custom_placement
import pandas as pd
import os,sys
from swarmI4.renderer.pygame_graphics.info import Info
import csv
import pyautogui
import  random
import  logging


class Swarm(object):
    """ This a wrapper for the all agents """

    def __init__(self,args, agent_generators: List[Tuple[int, Func]], placement_func: Func, my_map: Map):
        """ Create the swarm """
        # create an empty storage folder
        self.data_storage_dir = 'results'
        if not os.path.exists(self.data_storage_dir):
            os.mkdir(self.data_storage_dir)
        else:
            for f in os.listdir(self.data_storage_dir):
                os.remove(os.path.join(self.data_storage_dir, f))

        self._agents = []
        self.changed_agents_list = []
        self._positions = {}
        self.Time_Step = 1
        self.agent_done=0
        self.create_swarm(args,agent_generators, my_map, placement_func)
        self._my_map = my_map
        self.done = False
        self.success         = True

        #*****************************<Under Uncertainty>**************************************************
        self.consider_uncertainty  = False

        self.time_of_dynamic_event = 50
        self.uncertainty_rate      = 5     #% rate
        #change the map each time_of_dynamic_event step (use uncertainty_rate to calculate the number of moving obstacles)
        self.dynamic_OBS           = False
        self.add_new_OBS           = True
        #change the goal of k agents (k: calcukated using uncertainty_rate) each time_of_dynamic_event step (e.g: each 50 steps)
        self.goal_changing         = False
        # make delay for k agents
        self.agent_delayed         = False
        self.delay_timing           = 10

    def create_swarm(self,args, agent_generators: List[Tuple[int, Func]], my_map: Map, placement_func: Func) -> None:
        """ Create the swarm according to the generators """
        self.changed_agents_list.clear()
        self._agents.clear()
        my_map.new_OBS.clear()

        if args.agent_placement == "custom_placement":
            total_number_of_agents = len(my_map.custom_start_list)
        else:
            total_number_of_agents = sum([agent_type[0] for agent_type in agent_generators])

        for number, gen in agent_generators:
            for i in range(total_number_of_agents):
                position,targets= placement_func(i, total_number_of_agents, my_map)
                if args.agent_placement == "random_placement":
                   targets, _ = placement_func(i, total_number_of_agents, my_map)
                logging.info(f'position {position},{targets}')
                agent = gen(position)
                if targets is not None:
                    agent.target_list = [targets]
                    #my_map.set_as_target(targets)

                agent.id = i
                # set targets as they come in the custom map
                my_map.add_agent_to_map(agent)
                self._agents.append(agent)

        #for agent in self._agents:
           #if type(agent) is SmartAgent:
               #my_map.set_as_target(agent.target_list[0])

    def update_msg_box(self):
        for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.send_my_data(self._my_map)


    def agents_post_coordination(self):

        for i in range(0,5):
          self.update_msg_box()
          # post_negotiation
          for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.post_coordination(self._my_map)  # to solve the conflicts if two agents plan the same next_node

          self.update_msg_box()
          for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.post_negotiation(self._my_map)

        for i in range(6):
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


    def uncertainty(self):

        moving_agents = []
        for agent in self._agents:
            if type(agent) is SmartAgent and not agent.im_done and agent not in self.changed_agents_list:
                moving_agents.append(agent)

        num_agents_considered = int(self.uncertainty_rate * (len(self._agents) / 100))
        num_moving_OBS =  int(self.uncertainty_rate * self._my_map.number_of_obstacles / 100)


        #if num_agents_considered > len(moving_agents):  # the end of simulation
            #num_agents_considered = 1
        if len(moving_agents) == 1:
            num_agents_considered = 0

        if self.consider_uncertainty:

            new_target = None
            #agents_to_change_goals = []

            if self.goal_changing and (self.Time_Step % self.time_of_dynamic_event) == 0 and num_agents_considered > 0 and len(
                    moving_agents) > 0:
                # number of agent to change their goal locations
                for i in range(0, num_agents_considered):
                    random_agent = random.choice(moving_agents)
                    self.changed_agents_list.append(random_agent)
                    tries = 0
                    placed = False

                    while not placed and tries < 3000:
                        x, y = random.randint(0, self._my_map.size_x - 1), random.randint(0, self._my_map.size_y - 1)
                        if not self._my_map.occupied((x, y)) and not self._my_map.is_target((x, y)) and \
                            self._my_map._graph.nodes[(x, y)]["state"] == "free_space":
                            placed = True
                            new_target = (x, y)
                        else:
                            tries += 1
                    if new_target is not None:
                       random_agent.uncertainty(self._my_map,new_target , 0, False)
                    #moving_agents.remove(random_agent)  # to avoid selecting the same agent many times
                    #agents_to_change_goals.append(random_agent.id)

            # ********************<add delay to some agents plans>**************************************
            if self.agent_delayed and (self.Time_Step % self.time_of_dynamic_event) == 0 and num_agents_considered > 0 and len(moving_agents) > 0:
                # num_agents_considered = int(self.uncertainty_rate * (len(self._agents) / 100))
                agents_delayed = []
                for i in range(0, num_agents_considered):
                    random_agent = random.choice(moving_agents)
                    self.changed_agents_list.append(random_agent)
                    random_agent.uncertainty(self._my_map, None, self.delay_timing, False)
                    #moving_agents.remove(random_agent)  # to avoid selecting the same agent many times
                    agents_delayed.append(random_agent.id)
                #print("******************* List of Agent chosen randomly", agents_delayed)

            # ******** each time_of_dynamic_event remove a random obstacle and add a random obstacle **********************************
            if self.add_new_OBS :

              if self.Time_Step == 5 : # add the obstacles
               # check the previous new added obs
               if self._my_map.new_OBS is not None and len(self._my_map.new_OBS) > 0:
                      for obs in self._my_map.new_OBS:
                          self._my_map.set_as_free(obs)
                      self._my_map.new_OBS.clear()

               for i in range(0,num_moving_OBS):  #num_moving_OBS
                tries = 0
                placed = False
                free_node = None
                while not placed and tries < 3000:
                    x, y = random.randint(0, self._my_map.size_x - 1), random.randint(0, self._my_map.size_y - 1)
                    if not self._my_map.occupied((x, y)) and not self._my_map.is_target((x, y)) and \
                            self._my_map._graph.nodes[(x, y)]["state"] == "free_space":
                        placed = True
                        free_node = (x, y)
                    else:
                        tries += 1

                if free_node is not None:
                    self._my_map.set_as_obstacle(free_node)
                    if free_node not in self._my_map.new_OBS:
                        self._my_map.new_OBS.append(free_node)

                    for agent in self._agents:
                        if type(agent) is SmartAgent:
                            agent.uncertainty(self._my_map, None, 0, True)

              elif self.Time_Step % self.time_of_dynamic_event == 0: # move or add new one
                  # check the previous new added obs
                  if self._my_map.new_OBS is not None and len(self._my_map.new_OBS) > 0:
                      for obs in self._my_map.new_OBS:
                          self._my_map.set_as_free(obs)

                      self._my_map.new_OBS.clear()
                  # add new obstacles
                  for i in range(0, num_moving_OBS):  #num_moving_OBS
                      tries = 0
                      placed = False
                      free_node = None
                      while not placed and tries < 3000:
                          x, y = random.randint(0, self._my_map.size_x - 1), random.randint(0, self._my_map.size_y - 1)
                          if not self._my_map.occupied((x, y)) and not self._my_map.is_target((x, y)) and \
                                  self._my_map._graph.nodes[(x, y)]["state"] == "free_space":
                              placed = True
                              free_node = (x, y)
                          else:
                              tries += 1
                      if free_node is not None:
                          self._my_map.set_as_obstacle(free_node)
                          if free_node not in self._my_map.new_OBS:
                              self._my_map.new_OBS.append(free_node)

                          for agent in self._agents:
                              if type(agent) is SmartAgent:
                                  agent.uncertainty(self._my_map, None, 0, True)

            if self.dynamic_OBS and (self.Time_Step % self.time_of_dynamic_event) == 0 and num_moving_OBS > 0:
                for i in range(0, 1):  # num_moving_OBS
                    obs = self._my_map.get_random_OBS()
                    free_node = self._my_map.free_neighboring_node(obs, None)
                    if free_node is not None:
                        self._my_map.set_as_free(obs)
                        self._my_map.set_as_obstacle(free_node)
                        if obs in self._my_map.new_OBS:
                            self._my_map.new_OBS.remove(obs)
                        if free_node not in self._my_map.new_OBS:
                            self._my_map.new_OBS.append(free_node)

                        for agent in self._agents:
                            if type(agent) is SmartAgent:
                                agent.uncertainty(self._my_map, None, 0, True)



    def move_all(self,simulation_time,dt=0) -> None:
        """
        Move all agents in the swarm
        :world: The world
        :returns: None
        """
        logging.info(f'------------<New iteration started >-----------------------------')

        self.uncertainty()

        for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.next_step(self._my_map)

        # update msg box
        self.update_msg_box()

        # logging.info(f'Phase 02 : Handling conflicts ')
        for agent in self._agents:
            if type(agent) is SmartAgent:
                agent.handle_conflicts(self._my_map)

        self.agents_post_coordination()

        # logging.info(f'Phase 03 : AGVs are moving ...')
        for agent in self._agents:
          if type(agent) is SmartAgent:
            if not agent.im_done:#len(agent.remaining_path) > 0 :
               agent.move(self._my_map,simulation_time, time_lapsed=dt)
               #self.store_data(agent.storage_container,self.data_storage_dir,f'agent_{agent.id}.csv')

          else:
              agent.move(self._my_map, simulation_time, time_lapsed=dt)

        # update msg box
        #self.update_msg_box()

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
                       #logging.info(f'Agents failed in finding solutions to a deadlock')

                       #pyautogui.alert(text='Collision between : ' + str(agent1.id)+' from '+ str(agent1.last_node) + ' and ' +str(agent2.id) +' from ' +str(agent2.last_node), title='Conflict in node'+str(agent1.position),
                       #button='OK')


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
