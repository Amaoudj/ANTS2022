""" Install the package swarmI40 """
from setuptools import setup

setup(
    name='swarmI40',
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
