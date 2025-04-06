from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        #self.start_pos = [4.6, 2.4, 0]
        #self.end_pos = [1.6, 8, -pi/2]

        #self.start_pos2 = [4, 4, 0]
        #self.end_pos2 = [4, 8, 1.2*pi]

        #self.obs = [
        #    [2, 3, 6, 0.1],
        #    [2, 3, 0.1, 1.5],
        #    [4.3, 0, 0.1, 1.8],
        #    [6.5, 1.5, 0.1, 1.5],
        #    [0, 6, 3.5, 0.1],
        #    [5, 6, 5, 0.1]
        #]
        # Case Turtlebot3 - 4 rooms
        self.start_pos = [2, 2, 0]
        self.end_pos = [2, 17, 3*pi/4]

        self.start_pos2 = [4, 4, 0]
        self.end_pos2 = [4, 8, -pi]

        self.obs = [
            [0, 6, 6, 0.1],
            [6, 0, 0.1, 4],
            [0, 14, 4, 0.1],
            [6, 14, 0.1, 6],
            [14, 14, 6, 0.1],
            [14, 16, 0.1, 4],            
            [16, 6, 4, 0.1],
            [14, 0, 0.1, 6],
        ]
