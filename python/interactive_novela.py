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

	print('Which symbolic places? You have the choice between \
			\n-TABLE \n-DOOR \n-REST \n-SHELF')
	target = raw_input()



		else:
			print('This position doesn\'t exist')


else:
	pass
