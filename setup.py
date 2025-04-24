from setuptools import setup

setup(
    name='knowrob_ros',
    version='2.0.0',
    packages=[],                # no packages folders
    py_modules=['knowrob_ros_lib'],    # installs src/test_lib.py as module test_lib
    install_requires=['rospy'], # whatever ROS Python deps you have
    # you can also declare entry_points here if you want console_scripts
)
