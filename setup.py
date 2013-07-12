 #!/usr/bin/env python
 # -*- coding: utf-8 -*-

import os
from distutils.core import setup

setup(name='pyRobots',
      version='0.1',
      license='ISC',
      description='A collection of Python scripts to interface with PR2 ROS and GenoM modules',
      author='OpenRobots team',
      author_email='openrobots@laas.fr',
      package_dir = {'': 'src'},
      packages=['robots', 'robots.lowlevel', 'robots.actions', 'robots.helpers', 'robots.behaviours'],
      #scripts=['scripts/robots_test'], # Do not exist yet :-(
      data_files=[('share/pyrobots', ['share/' + f for f in ['pr2_postures.json', 'appart_places.json', 'jido_postures.json']]),
                  ('share/doc/pyrobots', ['AUTHORS', 'LICENSE', 'README'])]
      )
