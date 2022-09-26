from setuptools import setup
import sys

if len(sys.argv) < 2 :
    print('Argument --sim for simulator, --dummy for dummy and --real for real robot.')
    exit(1)

if '--sim' in sys.argv:
    sys.argv.remove('--sim' )
    cur_packages = ['fossbot_lib/coppeliasim_robot/',
                    'fossbot_lib/common',
                    'fossbot_lib/parameters_parser']
    with open('fossbot_lib/coppeliasim_robot/requirements.txt') as f:
        requirements = f.read().splitlines()
elif '--real' in sys.argv:
    raise NotImplementedError
elif '--dummy' in sys.argv:

    sys.argv.remove('--dummy')
    cur_packages = ['fossbot_lib/dummy_robot/',
                    'fossbot_lib/common',
                    'fossbot_lib/parameters_parser']

    with open('fossbot_lib/dummy_robot/requirements.txt') as f:
        requirements = f.read().splitlines()
else:
    print('Argument not found')
    exit()

setup(
   name='fossbot_lib',
   version='0.1.0',
   author='Christos Chronis',
   author_email='chronis@hua.gr',
   packages= cur_packages,
#    scripts=['bin/script1','bin/script2'],
#    url='http://pypi.python.org/pypi/PackageName/',
#    license='LICENSE.txt',
   description='An awesome package that does something',
#    long_description=open('README.txt').read(),
   install_requires= requirements
 )
