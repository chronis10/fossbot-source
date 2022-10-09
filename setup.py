from setuptools import setup
import sys
from setuptools.command.install import install as _install

cur_packages = ['fossbot_lib/common/data_structures',
                'fossbot_lib/common/interfaces',
                'fossbot_lib/parameters_parser']

requirements = []
mydata = {'':[]}
class Install(_install):
    user_options = _install.user_options + [('platform=', None, None)]
    def initialize_options(self):
        _install.initialize_options(self)
        self.platform = None

    def finalize_options(self):
        _install.finalize_options(self)
    
    def load_requirements(self,path):
        with open(path) as f:
            requirements = f.read().splitlines()
        return requirements

    def run(self):
        global cur_packages,requirements,mydata
        if self.platform == "sim":
            cur_packages.append('fossbot_lib/coppeliasim_robot/')
            requirements = self.load_requirements('fossbot_lib/coppeliasim_robot/requirements.txt')
            mydata = {'fossbot_lib/coppeliasim_robot/':['lib/Linux/remoteApi.so','lib/Windows/remoteApi.dll','lib/MacOS/remoteApi.dylib']}
        elif self.platform == "real":
            cur_packages.append('fossbot_lib/real_robot/')
            requirements = self.load_requirements('fossbot_lib/real_robot/requirements.txt')
        else:
            cur_packages.append('fossbot_lib/dummy_robot/')
            requirements = self.load_requirements('fossbot_lib/real_robot/requirements.txt')
        _install.run(self)

setup(
   name='fossbot_lib',
   version='0.1.0',
   author='Christos Chronis & Manousos Linardakis',
   author_email='chronis@hua.gr',
   packages= cur_packages,
#    scripts=['bin/script1','bin/script2'],
#    url='http://pypi.python.org/pypi/PackageName/',
#    license='LICENSE.txt',
   description='An awesome package that does something',
#    long_description=open('README.txt').read(),
   package_data = mydata ,
   install_requires= requirements,
   cmdclass={'install': Install}
 )
