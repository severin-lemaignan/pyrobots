 #!/usr/bin/env python
 # -*- coding: utf-8 -*-

import os
from distutils.core import setup

#import __version__
execfile('src/robots/__init__.py')


setup(name='pyrobots',
      version=__version__,
      license='ISC',
      description='A collection of Python scripts to execute and manages tasks for robots',
      author='Séverin Lemaignan',
      author_email='severin.lemaignan@epfl.ch',
      package_dir = {'': 'src'},
      packages=['robots', 'robots.helpers', 'robots.mw'],
      scripts=['bin/robot_introspection'], 
      data_files=[
                  ('share/doc/pyrobots', ['AUTHORS', 'LICENSE', 'README.md'])]
      )
