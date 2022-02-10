from typing import Callable as Func, List, Tuple
from ..map import Map
from ..agent import agent_placement
from swarmI4.agent.smart_agent import SmartAgent
from swarmI4.agent.agent_placement import custom_placement
import pandas as pd
import os,sys
from swarmI4.renderer.pygame_graphics.info import Info
import csv
import ctypes
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
        self._positions = {}

        self.create_swarm(args,agent_generators, my_map, placement_func)
        self._my_map = my_map



    def create_swarm(self,args, agent_generators: List[Tuple[int, Func]], my_map: Map, placement_func: Func) -> None:
        """ Create the swarm according to the generators """

        self._agents.clear()
        if args.agent_placement == "custom_placement":
            total_number_of_agents = len(my_map.custom_start_list)
        else:
            total_number_of_agents = sum([agent_type[0] for agent_type in agent_generators])

        for number, gen in agent_generators:
            for i in range(total_number_of_agents):
                position,targets= placement_func(i, total_number_of_agents, my_map)
                logging.info(f'position {position},{targets}')
                agent = gen(position)
                if targets is not None:
                    agent.target_list = [targets]
                agent.id = i
                # set targets as they come in the custom map
                my_map.add_agent_to_map(agent)
                self._agents.append(agent)


    def move_all(self,simulation_time,dt=0) -> None:

        """
        Move all agents in the swarm
        :world: The world
        :returns: None
        """
        print(f'------------< iteration started >-----------------------------')
        for agent in self._agents:
            if type(agent) is SmartAgent:

                #logging.info(f'Phase 01 : planning the next step ')
                agent.next_step(self._my_map)

        for agent in self._agents:
            if type(agent) is SmartAgent:
                #logging.info(f'Phase 02 : Handling rising conflicts ')
                num_pos_requests, _ = agent.pos_requests(agent.position)
                #logging.info(f'agent: {agent.id}, requests : {num_pos_requests}')
                agent.handle_conflicts(self._my_map)


        for agent in self._agents:
          if type(agent) is SmartAgent:
            #logging.info(f'Phase 03 : AGVs are moving ...')
            if agent.remaining_nodes >0 :
               agent.move(self._my_map,simulation_time, time_lapsed=dt)
               self.store_data(agent.storage_container,self.data_storage_dir,f'agent_{agent.id}.csv')

          else:
              agent.move(self._my_map, simulation_time, time_lapsed=dt)

        print('------------<iteration ended>-----------------------------')
        #clear this list for the next use
        self._my_map.new_paths_node.clear()


        #TODO: I need to add a func to reset and run the simulation again
        for agent1 in self._agents:
            for agent2 in self._agents:
                if agent1.id != agent2.id and agent1.position == agent2.position :
                    ctypes.windll.user32.MessageBoxW(0,
                                                     f"Collision in node {agent1.position} between : {agent1.id} and {agent2.id}",
                                                     "Conflict", 1)
                    sys.exit()


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
