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


    def setup(self,args):
        """
        Set up the rendering. Called before every new simulation.
        """
        self.sim_info_display(step=0, lapsed_time=0, sim_arguments=args)


    def tear_down(self):
        """
        Tear down the rendering. Called after every simulation.
        """
    def display_frame(self,args=None, step: int=0,lapsed_time: float = 0):
        """
        Display a frame
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise SystemExit

        self._display.reset_canvas()
        for node in self._graph.nodes:
            self._display.draw_node(node_pos=node,graph=self._my_map.graph)
        self._display.draw_grid_lines(self._my_map.size_xy)
        self._display.draw_selections(self._swarm.agents)
        self._display.draw_targets(self._swarm.agents)
        self._display.draw_agents(self._swarm.agents)
        self._display.plot_stats()
        self.sim_info_display(step, lapsed_time, sim_arguments = args)
        self._display.write_info(self._sim_info)
        pygame.display.update()

    def sim_info_display(self, step, lapsed_time: float = 0, sim_arguments=None):
        self._sim_info = []
        self._sim_info.append(Info(f'Simulation Info', is_title=True))
        self._sim_info.append(Info(f'GRAPH SIZE: {sim_arguments.nodes}'))
        self._sim_info.append(Info(f'NUMBER OF AGVs: {sim_arguments.swarm_size}'))
        self._sim_info.append(Info(f'NUMBER OF TARGETS OF EACH AGV: {sim_arguments.num_targets}'))
        self._sim_info.append(Info(f'Simulation Statistics', is_title=True))
        self._sim_info.append(Info(f'Step: {step}'))
        self._sim_info.append(Info(f'Lapsed Time: {round(lapsed_time, 1)}  sec'))


    @property
    def display(self):
        return self._display






