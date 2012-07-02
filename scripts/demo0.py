import robots; pr2 = robots.PR2(init=False)
#from subprocess import Popen


pr2.setpose("TRANSITION")
pr2.setpose("WAIT")

#p1 = Popen(["rosrun", "sound_play", "say.py", "welcome to the new building"])
pr2.say("welcome to the new building!")
pr2.setpose("WELCOME")
pr2.setpose("TUCK_LARM")
#p2 = Popen(["rosrun", "sound_play", "say.py", "please, follow me"])
pr2.say("please, follow me")
pr2.setpose("MANIP")
pr2.waypoints([(12.882074, 15.706191, 0, 0, 0, -2.407687),(10.081528, 16.044288, 0, 0, 0,  3.120482), (7.333865,  14.687783, 0, 0, 0, -3.134351), (1.475024,  13.766173, 0, 0, 0, -2.214792), (1.312933,  4.420253,  0, 0, 0, -1.214531), (5.397113,  4.071713,  0, 0, 0,  1.008833), (7.719524,  6.846402,  0, 0, 0, -3.089865)])
pr2.setpose("HAND_SWEEP1")
#p3 = Popen(["rosrun", "sound_play", "say.py", "nice to meet you"])
pr2.say("Here we are!")
pr2.setpose("HAND_SWEEP2")
pr2.setpose("MANIP")

#p1.terminate()
#p2.terminate()
#p3.terminate()
pr2.close();
