from math import pi


class TestCase:
    """ Defined the map for our assignment """

    def __init__(self):

        # self.start_pos = [1, 8, 0]
        # self.end_pos = [3, 8, 0]
        self.start_pos = [5, 5, 0]
        self.end_pos = [6, 5, 3*pi/4]


        # [x_position, y_position, x_width, y_width]
        # Discovery: the x-position and y-position are the bottom left corner of the box
        # If the map looks off, the coordinates of the boxes are probably wrong
        self.obs = [
            [3.3, 0, 2, 0.2],   # box_wall_right
            [0, 1, 0.5, 0.2],   # box_wall_left
            [1.45, 0.7, 0.5, 0.2],    # box_wp1
            [3.16, 0.7, 0.5, 0.2],   # box_wp2
            [1.2, 1.65, 0.2, 0.4],   # random_1
            [2.3, 1.65, 0.4, 0.4],   # random_2
            [3.61, 1.95, 0.4, 0.2],  # box_wp6
        ]
