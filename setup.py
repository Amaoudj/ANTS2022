""" Install the package swarmI4 """
from setuptools import setup

setup(
    name='swarmI4',
    version='0.1.0',
    author='Anders Lyhne Christensen',
    author_email='anderslyhne@gmail.com',
    packages=[
        'swarm',
        'swarm.map',
        'swarm.agents'
    ],
    install_requires=[
        "numpy",
        "networkx",
        "matplotlib",
        "pygraphviz"
    ],
)
