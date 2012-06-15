import os
import glob
import os
import json
import logging; logger = logging.getLogger("robot." + __name__)
logger.setLevel(logging.DEBUG)

#hack that tries to find out the current prefix and then the data directory
DATA_DIR = os.path.abspath(__file__).split('lib')[0] + 'share/trajectories/Seg_file/'

SUFFIX = '_Seg.traj'



class Trajectory:

    def __init__(self, traj):
        self.name = traj
        self.path = os.path.abspath(os.path.join(PREFIX, self.name + SUFFIX))
        if not os.path.isfile(self.path):
            raise NameError("Trajectory " + traj + " do not exist")

    def abspath(self):
        return self.path

    def initcoords(self):

        with open (self.path,'r') as f:
            for i in range(0,3):
                line = f.readline()

        return [float(coord) for coord in line.split()]

    @staticmethod
    def list():
        # return a list of all available trajectories
        return []

if __name__ == "__main__" :

    traj = Trajectory("gym_3")

    print(" Abs path: " + traj.abspath())
    print(" Init coords: " + str(traj.initcoords()))

    for t in Trajectory.list():
        print(" Name: " + t.name)
        print(" Abs path: " + t.abspath())
        print(" Init coords: " + str(t.initcoords()))

