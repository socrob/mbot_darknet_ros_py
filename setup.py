from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['ros/scripts/darknet_ros_py_node'],
    packages=['darknet_ros'],
    package_dir={'darknet_ros': 'ros/src/darknet_ros'}
)
