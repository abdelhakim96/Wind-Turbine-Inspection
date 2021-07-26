# CMake generated Testfile for 
# Source directory: /home/jonas/code/catkin_Windturbine/src/mavros_controllers/geometric_controller
# Build directory: /home/jonas/code/catkin_Windturbine/src/mavros_controllers/geometric_controller/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_geometric_controller_gtest_geometric_controller-test "/home/jonas/code/catkin_Windturbine/src/mavros_controllers/geometric_controller/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/jonas/code/catkin_Windturbine/src/mavros_controllers/geometric_controller/cmake-build-debug/test_results/geometric_controller/gtest-geometric_controller-test.xml" "--return-code" "/home/jonas/code/catkin_Windturbine/src/mavros_controllers/geometric_controller/cmake-build-debug/devel/lib/geometric_controller/geometric_controller-test --gtest_output=xml:/home/jonas/code/catkin_Windturbine/src/mavros_controllers/geometric_controller/cmake-build-debug/test_results/geometric_controller/gtest-geometric_controller-test.xml")
set_tests_properties(_ctest_geometric_controller_gtest_geometric_controller-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/melodic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/melodic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/jonas/code/catkin_Windturbine/src/mavros_controllers/geometric_controller/CMakeLists.txt;62;catkin_add_gtest;/home/jonas/code/catkin_Windturbine/src/mavros_controllers/geometric_controller/CMakeLists.txt;0;")
subdirs("gtest")
