import pygame
from swarmI4.renderer.pygame_graphics.pygame_renderer import PygameRenderer

class EventHandler:
    def __init__(self):
        self.mouse_drag = False
        self.goal_flag = False
        self.selected_agent = None
        self.sim_start = False # start simulation ?
        self._map = None

    def handle_events(self, event, display, map, agents):
        self._map = map
        graph = self._map.graph
        if self._handle_agent_selection(display, agents):
            return
        if self.selected_agent is not None:
            if pygame.mouse.get_pressed(3)[2]:
                self._move_agent(display)
            if pygame.mouse.get_pressed(3)[0]:
                self._set_target(display)

        # set or remove obstacles by click left or right mouse buttons
        elif pygame.mouse.get_pressed(3)[0] or pygame.mouse.get_pressed(3)[2]:
            self._handle_obstacle_setting(display)

        # start or pause simulation by pressing enter key
        elif event.type==pygame.KEYDOWN:
            if event.key==pygame.K_RETURN:
                self.sim_start = not self.sim_start

    def _set_obs(self, diplay)->None:
        selected_node = self._get_selected_node(diplay)
        self._map.set_as_obstacle(selected_node)

    def _set_free_space(self,diplay)->None:
        if pygame.mouse.get_pressed(3)[2]:
            selected_node = self._get_selected_node(diplay)
            self._map.set_as_free(selected_node)

    def _check_target_key(self):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_g]:
            self.goal_flag = True


    def _get_selected_node(self,diplay)->tuple:
        """
        get the node in the cursor position
        :param diplay: the display object
        :param graph: the graph
        :return: the node object
        """
        mouse_pos = pygame.mouse.get_pos()
        node_grid_coords = diplay.to_grid_coords(mouse_pos)

        return node_grid_coords

    def _is_agent_selected(self, diplay, agents):
        """
        method for returning the agent currently being selected
        or None if no agent is selected
        :param display
        :param graph
        :param agents
        :return: the agent object
        """
        selected_node = self._get_selected_node(diplay)

        for agent in agents:
            if selected_node == agent.position:
                return agent
        return None

    def _handle_obstacle_setting(self, diplay):
        # flag for indicating that mouse is being dragged
        self.mouse_drag = True
        # set the current node as obstacle or free space
        if self.selected_agent is None:
            if pygame.mouse.get_pressed(3)[0]:
                self._set_obs(diplay)
            # set the current node as free space
            elif pygame.mouse.get_pressed(3)[2]:
                self._set_free_space(diplay)

    def _move_agent(self,diplay):
        graph = self._map.graph
        selected_node = self._get_selected_node(diplay)
        print(graph,selected_node,graph[selected_node])
        print(graph.nodes)
        if graph.nodes[selected_node]["state"] == 'free_space':
            self.selected_agent.position = selected_node


    def _set_target(self,map):
        # select target for the selected agent and add it to its target list
        if self.selected_agent is not None:
            selected_node = self._get_selected_node(map)
            self._map.nodes[selected_node]["state"] = 'target'
            self.selected_agent.target_list.append(selected_node)


    def _handle_agent_selection(self, display, agents):
        """
        method for selecting and un-selecting agents using the
        mouse left click
        :param display:
        :param graph:
        :param agents:
        :return:
        """

        if pygame.mouse.get_pressed(3)[0]:
            agent = self._is_agent_selected(display, agents)
            if agent is None:
                return False
            else:
                if agent is self.selected_agent and self.selected_agent is not None:
                    self.selected_agent = None
                    agent.is_selected = False
                    print(f'you have deselected the agent: {agent.id}')
                elif agent is not self.selected_agent:
                    self.selected_agent = agent
                    agent.is_selected = True
                    print(f'you have selected the agent: {agent.id}')
                return True

    def handle_init_events(self,args,renderer,map,swarm):
        if type(renderer) is PygameRenderer:
            while not self.sim_start:
                renderer.display_frame(args)
                for e in pygame.event.get():
                    if e.type == pygame.QUIT:
                        quit()
                    else:
                        self.handle_events(e,
                                            renderer.display,
                                            map,
                                            swarm.agents)
        else:
            pass




