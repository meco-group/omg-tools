import os
from setuptools import setup


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="motionplanning",
    version="0.1",
    author="Ruben Van Parys & Tim Mercy",
    author_email="ruben.vanparys@kuleuven.be",
    description=('spline-based motion planning toolbox for modeling, ' +
                 'simulating and embedding motion planning algorithms'),
    # license="LGPLv3",
    keywords="optimization motion planning",
    # url="",
    packages=['motionplanning'],
    long_description=read('readme.md'),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        # "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
    ],
)
