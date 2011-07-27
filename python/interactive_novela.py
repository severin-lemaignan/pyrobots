#! /usr/bin/env python

import json
import sys

f = open('share/novela_places.json','rb')
json_data=f.read()
symbolic_places = json.loads(json_data)

print('Welcome to the interactive interface of the Novela project\
	\nWhat do y ou want to do?\
	\n-Tape 1 to use ros_nav')

choice = raw_input()

if choice == '1':

	print('Which symbolic places?')

	target = raw_input()

	for key, value in symbolic_places.items():
	
		if key == target:
			#print ('%s is at (%d, %d, %d, %d, %d, %d, %d,)' % (key, value['a'], value['b'], value['c'], value['d'], value['e'], value['f'], value['g']))
			 position, x, y, z, qx, qy, qz, qw = 
                    [ key , \
					value['a'], value['b'], value['c'], \
                    value['d'], value['e'], value['f'], value['g']
                    ]

		else:
			print('This position doesn\'t exist')


else:
	pass
