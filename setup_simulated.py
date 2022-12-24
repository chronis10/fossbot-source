from setuptools import setup

setup(
   name='fossbot_lib',
   version='0.1.1',
   author='Christos Chronis & Manousos Linardakis',
   author_email='chronis@hua.gr',
   packages= ['fossbot_lib/common/data_structures','fossbot_lib/common/interfaces','fossbot_lib/parameters_parser','fossbot_lib/coppeliasim_robot/'],
#    scripts=['bin/script1','bin/script2'],
#    url='http://pypi.python.org/pypi/PackageName/',
#    license='LICENSE.txt',
   description='FossBot Simulator Library')
#    long_description=open('README.txt').read(),
   #package_data ={'fossbot_lib/coppeliasim_robot/':['lib/Linux/remoteApi.so','lib/Windows/remoteApi.dll','lib/MacOS/remoteApi.dylib']},
#    install_requires= ['fossbot_lib/coppeliasim_robot/requirements.txt'])
