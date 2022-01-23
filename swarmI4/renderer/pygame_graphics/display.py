import numpy as np
import pygame
import matplotlib
from   matplotlib import pyplot as plt
import matplotlib.backends.backend_agg as agg
from   swarmI4.agent.smart_agent import SmartAgent
import random
import networkx as nx
import os

###COLORS
BLUE_LIGHT = (89, 181, 235)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 150, 255)




class Display:
    def __init__(self, display_size:tuple, resolution:int)->None:
        """
        a class for creating the simulation map
        """

        self.resolution = resolution
        self.height, self.width = display_size

        self.nodes_states = {'free_space':BLACK,
                             'obstacle':GREEN,
                             'start': None,
                             'target':WHITE,
                             'path':BLACK,#BLUE_LIGHT,
                             'agent':WHITE}

        pygame.init()
        self.name = 'Multi-AGV Simulator'
        pygame.display.set_caption(self.name)
        self.canvas = pygame.display.set_mode((self.width, self.height))
        self.text_location = self.width-self.width/3, 10
        self.text_font_path = "swarmI4/renderer/pygame_graphics/resources/Fonts/Roboto"

    def reset_canvas(self):
        self.canvas.fill(BLACK)

    def draw_grid_lines(self,grid_dimensions:tuple)->None:
        """
        method for drawing the grid lines
        :return:
        """
        rows, cols = grid_dimensions
        width_px, height_px = cols * self.resolution, rows*self.resolution

        for i in range(rows + 1):
            pygame.draw.line(self.canvas, GREEN, (0, i * self.resolution ), (width_px, i * self.resolution))
        for j in range(cols + 1):
            pygame.draw.line(self.canvas, GREEN, (j * self.resolution, 0), (j * self.resolution, height_px))
        pygame.draw.rect(self.canvas, GREEN, [0, 0, height_px, height_px], 3)

    def draw_node(self, node_pos:tuple, graph:nx)->None:
        """
        method for drawing a node in the canvas
        :param node_pos: the node position in the grid (row,col)
        :param node_state: the node state (free_space , obstacle .... etc)
        :return:
        """

        node_state = graph.nodes[node_pos]["state"]
        color      = self.nodes_states[node_state]
        row,col    = node_pos
        x,y = col * self.resolution, row * self.resolution
        pygame.draw.rect(self.canvas, color, (x, y, self.resolution, self.resolution))


    def to_grid_coords(self, canvas_coords):
        """
        convert canvas coords (x_px, y_px) to grid coords (row,col)
        :param canvas_coords: canvas coordinates (x_px, y_px)
        :return: grid coordinates (row,col)
        """
        col = canvas_coords[0]//self.resolution
        row = canvas_coords[1]//self.resolution
        return row, col


    def draw_agents(self, agents:list)->None:
        """
        method for drawing an agent in the canvas
        :param agent: the agent object
        :return:
        """
        for agent in agents:
            row,col = agent.position

            width, height = agent.size[0]*self.resolution,agent.size[1]*self.resolution
            x_px, y_px = col * self.resolution, row * self.resolution
            if agent.orientation == 0:
                pygame.draw.rect(self.canvas, RED,[x_px, y_px,self.resolution,self.resolution])
                pygame.draw.rect(self.canvas, WHITE,[x_px+width/2, y_px+height/4,width/3,height/2])
            elif agent.orientation == 1:
                pygame.draw.rect(self.canvas, RED,[x_px, y_px,self.resolution,self.resolution])
                pygame.draw.rect(self.canvas, WHITE,[x_px+width/4, y_px+height/4,width/2,height/3])
            elif agent.orientation == 2:
                pygame.draw.rect(self.canvas, RED,[x_px, y_px,self.resolution,self.resolution])
                pygame.draw.rect(self.canvas, WHITE,[x_px+width/4, y_px+height/4,width/3,height/2])
            elif agent.orientation == 3:
                pygame.draw.rect(self.canvas, RED,[x_px, y_px,self.resolution,self.resolution])
                pygame.draw.rect(self.canvas, WHITE,[x_px+width/4, y_px+height/2,width/2,height/3])


    def draw_targets(self,agents:list)->None:
        """
        draw all the targets of the agents
        :param agents: a list of all the agents
        :return:
        """
        for agent in agents:
            if type(agent) is SmartAgent:
                targets = agent.target_list[agent.target_id:]
                for target in targets:
                    row,col = target
                    x_px, y_px = col * self.resolution, row * self.resolution
                    pygame.draw.rect(self.canvas, BLUE_LIGHT, [x_px, y_px, self.resolution, self.resolution])#WHITE
                    pygame.draw.circle(self.canvas,WHITE, (x_px + self.resolution / 2, y_px + self.resolution / 2),radius=self.resolution/2,width=0)


    def draw_selections(self, agents:list)->None:
        """
        draw a box around a selected agent
        :param agents: list of agents
        :return:
        """
        for agent in agents:
            if agent.is_selected is True:
                row, col = agent.position
                x_px, y_px = col * self.resolution, row * self.resolution
                #pygame.draw.rect(self.canvas, WHITE, [x_px, y_px, self.resolution, self.resolution],5)
                pygame.draw.circle(self.canvas,
                                   RED,
                                   (x_px+self.resolution/2, y_px+self.resolution/2),
                                   radius=self.resolution/1.5,
                                   width=2)
                for target in agent.target_list:
                    target_row,target_col = target
                    target_x_px, target_y_px = target_col * self.resolution, target_row * self.resolution
                    pygame.draw.line(self.canvas,RED,(x_px, y_px),(target_x_px,target_y_px),2 )
                    pygame.draw.circle(self.canvas,
                                       RED,
                                       (target_x_px + self.resolution/2, target_y_px + self.resolution / 2),
                                       radius=self.resolution/1.5,
                                       width=2)

            else:
                pass

    def write_info(self,info):
        empty_space = 0

        for i,element in enumerate(info):
            if element.is_title:
                empty_space += 10
                font_path = os.path.join(self.text_font_path,'Roboto-Bold.ttf')
                font = pygame.font.Font(font_path, 25)
                text = font.render(element.text, True, WHITE, BLACK)
                textRect = text.get_rect()
                textRect.topleft = self.text_location[0], self.text_location[1] + 20 *i  + empty_space
                empty_space += 30

                self.canvas.blit(text,textRect)
            else:
                font_path = os.path.join(self.text_font_path, 'Roboto-Light.ttf')
                font = pygame.font.Font(font_path, 14)
                text = font.render(element.text, True, WHITE, BLACK)
                textRect = text.get_rect()
                textRect.topleft = self.text_location[0], self.text_location[1] + 20*i + empty_space
                self.canvas.blit(text, textRect)

    def plot_stats(self):
        data = random.sample(range(10, 100), 80)
        time = np.linspace(0, 1, 80)

        matplotlib.use("Agg")

        plt.style.use('dark_background')

        fig = plt.figure(1)
        fig.set_size_inches(3, 3)

        plt.plot(time, data)
        canvas = agg.FigureCanvasAgg(fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.tostring_rgb()
        size = canvas.get_width_height()
        surf = pygame.image.fromstring(raw_data, size, "RGB")
        plt.clf()
        self.canvas.blit(surf, (760, 200))
        pygame.display.flip()

        return surf


