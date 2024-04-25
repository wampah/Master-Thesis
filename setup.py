from setuptools import setup,find_packages
setup(name='five_bar',
      version='0.1',
      packages=find_packages(exclude=['mthesis_env']),
      install_requires=['gymnasium',
                       'numpy']) # And any other dependencies required