#! /usr/bin/env python

import logging
logger = logging.getLogger("robot")
console = logging.StreamHandler()
console.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
console.setFormatter(formatter)
logger.addHandler(console)

import unittest
import position

import robots

class TestPositions(unittest.TestCase):

    def setUp(self):
        robot = robots.Robot(dummy = True)
        self.poses = position.PoseManager(robot)
    
    def test_normalize(self):
        
        ref = {'x':1.2, 'y':2.1, 'z':1.8, 'qx':0.1, 'qy':0.0, 'qz':0.5, 'qw':1.0, 'frame':'map'}
        ref0 = {'x':0.0, 'y':0.0, 'z':0.0, 'qx':0.0, 'qy':0.0, 'qz':0.0, 'qw':0.0, 'frame':'map'}
        ref1 = {'x':0.0, 'y':0.0, 'z':0.0, 'qx':None, 'qy':None, 'qz':None, 'qw':None, 'frame':'map'}
        ref2 = {'x':None, 'y':None, 'z':None, 'qx':0.0, 'qy':0.0, 'qz':0.0, 'qw':0.0, 'frame':'map'}
        
        self.assertEqual(ref0, self.poses.get(ref0))
        self.assertEqual(ref, self.poses.get(ref))
        
        self.assertEqual(ref1, self.poses.get([0,0,0]))
        self.assertEqual(ref0, self.poses.get([0,0,0,0,0,0]))
        self.assertEqual(ref0, self.poses.get([0,0,0,0,0,0,0]))
        
        self.assertEqual(ref, self.poses.get([1.2,2.1,1.8,0.1,0.0,0.5,1.0]))
        
        
        self.assertEqual(ref1, 
                         self.poses.get({'x':0.0, 'y':0.0, 'z':0.0}))
        self.assertEqual(ref2, 
                         self.poses.get({'qx':0.0, 'qy':0.0, 'qz':0.0, 'qw':0}))
        
        
        
    def test_ros(self):
        print(self.poses.get("base_link"))
        pose = self.poses.get("map")
    
    def test_spark(self):
        pose = self.poses.get("HERAKLES_HUMAN1")
        pose = self.poses.get(("HERAKLES_HUMAN1", "Pelvis"))
        pose = self.poses.get(("HERAKLES_HUMAN1", "HeadX"))
        
        pose = self.poses.get("PR2_ROBOT")

def test_suite():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestPositions)
    #suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestSentence))
    return suite
    
if __name__ == '__main__':
    import sys
    if sys.version_info >= (2,7,0):
        unittest.TextTestRunner(failfast = True).run(test_suite())
    else:
        print ("Running Python < 2.7. Failsafe mode not possible.")
        unittest.TextTestRunner().run(test_suite())
