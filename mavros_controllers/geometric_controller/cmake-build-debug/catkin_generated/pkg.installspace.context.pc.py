# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;mavros_msgs;geometry_msgs;sensor_msgs;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgeometric_controller".split(';') if "-lgeometric_controller" != "" else []
PROJECT_NAME = "geometric_controller"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
