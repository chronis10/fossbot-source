from setuptools import setup
import os

setup(
   name='fossbot_lib_godot',
   version= os.getenv('PYPI_VERSION').replace('v','').strip(),
   author='Christos Chronis & Manousos Linardakis',
   author_email='chronis@hua.gr',
   packages= ['fossbot_lib/common/data_structures','fossbot_lib/common/interfaces','fossbot_lib/parameters_parser','fossbot_lib/godot_robot/'],
   description='FossBot Godot Simulator Library')
