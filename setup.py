 #!/usr/bin/env python
 # -*- coding: utf-8 -*-

import os
from distutils.core import setup

#import __version__
execfile('src/robots/__init__.py')


setup(name='pyRobots',
      version=__version__,
      license='ISC',
      description='A Python toolset for event-based, asynchronous programming of robot controllers',
      classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: ISC License (ISCL)',
        'Programming Language :: Python :: 2.7',
        'Topic :: Scientific/Engineering',
      ],
      author='Séverin Lemaignan',
      author_email='severin.lemaignan@epfl.ch',
      url='https://github.com/chili-epfl/pyrobots',
      install_requires=["futures", "numpy"],
      package_dir = {'': 'src'},
      packages=['robots', 'robots.helpers', 'robots.mw', 'robots.concurrency', 'robots.events', 'robots.poses', 'robots.resources'],
      scripts=['bin/robot_introspection'], 
      data_files=[
                  ('share/doc/pyrobots', ['AUTHORS', 'LICENSE', 'README.md'])]
      )
