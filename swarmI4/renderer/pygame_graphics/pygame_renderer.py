from swarmI4.renderer.renderer_interface import RendererInterface
from swarmI4.renderer.pygame_graphics.display import Display
import pygame
from swarmI4.renderer.pygame_graphics.info import Info

class PygameRenderer (RendererInterface):
    """
    A simple pygame-renderer
    """
    def __init__(self,args,my_map, swarm):
        super().__init__(my_map, swarm)
        self._display_initialized = False # used later
        self._display = Display(args.display_size,args.resolution)
        self._sim_info = []
        self._graph = self._my_map
        self.new_added_obs_node = []


    def setup(self,args):
        """
        Set up the rendering. Called before every new simulation.
        """
        self.sim_info_display(step=0, lapsed_time=0, sim_arguments=args)


    def tear_down(self):
        """
        Tear down the rendering. Called after every simulation.
        """
    def display_frame(self,args=None, step: int=0,lapsed_time: float = 0) -> str or None:
        """
        Display a frame
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return 'stop'
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    return 'reset'

        self._display.reset_canvas()
        for node in self._graph.nodes:
            self._display.draw_node(node_pos=node,graph=self._my_map.graph)

        #*******************************<New added obstacles>********************************************
        if self.new_added_obs_node is not None and len(self.new_added_obs_node) > 0:
            for node in self._graph.nodes:
                self._display.draw_node(node_pos=node, graph=self._my_map.graph)
            self.new_added_obs_node.clear()
        if self._my_map.new_OBS is not None :
          for node in self._my_map.new_OBS:
            self.new_added_obs_node.append(node)
            self._display.draw_new_added_node(node_pos=node,graph=self._my_map.graph)
        #******************************************************************************

        self._display.draw_grid_lines(self._my_map.size_xy)
        self._display.draw_selections(self._swarm.agents)
        self._display.draw_targets(self._swarm.agents)
        self._display.draw_agents(self._swarm.agents)
        #self._display.plot_stats()
        self.sim_info_display(step, lapsed_time, sim_arguments = args)
        #self._display.write_info(self._sim_info)
        pygame.display.update()
        return None

    def sim_info_display(self, step, lapsed_time: float = 0, sim_arguments=None):
        self._sim_info = []
        self._sim_info.append(Info(f'Simulation Info', is_title=True))
        self._sim_info.append(Info(f'GRAPH SIZE: {self._my_map.size_xy}'))
        self._sim_info.append(Info(f'NUMBER OF AGVs: {len(self._swarm.agents)}'))
        self._sim_info.append(Info(f'NUMBER OF TARGETS OF EACH AGV: {self._my_map.number_of_targets}'))
        self._sim_info.append(Info(f'Simulation Statistics', is_title=True))
        self._sim_info.append(Info(f'Step: {step}'))
        self._sim_info.append(Info(f'Lapsed Time: {round(lapsed_time, 1)}  sec'))


    @property
    def display(self):
        return self._display






