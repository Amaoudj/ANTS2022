""" The interface for a renderer """


class RendererInterface:
    """ Interface class for a renderr """

    def __init__(self, my_map, swarm):
        """ Create the Renderer
        :my_map: The map to use
        :swarm: The swarm to simulate
        """

        self._my_map = my_map
        self._swarm  = swarm

    def setup(self,args:None):
        """
        Set up the rendering. Called before every new simulation.
        """
        return NotImplementedError("Swarm agents should contains this method")

    def tear_down(self):
        """
        Tear down the rendering. Called after every simulation.
        """
        return NotImplementedError("Swarm agents should contains this method")

    def display_frame(self, args=None, step: int = 0, lapsed_time: float = 0):
        """
        Display a frame
        """
        return NotImplementedError("Swarm agents should contains this method")

