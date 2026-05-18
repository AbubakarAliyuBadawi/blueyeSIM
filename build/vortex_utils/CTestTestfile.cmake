# CMake generated Testfile for 
# Source directory: /home/badawi/Desktop/auto-pilot/src/vortex-utils
# Build directory: /home/badawi/Desktop/auto-pilot/build/vortex_utils
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_utils "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/badawi/Desktop/auto-pilot/build/vortex_utils/test_results/vortex_utils/test_utils.xunit.xml" "--package-name" "vortex_utils" "--output-file" "/home/badawi/Desktop/auto-pilot/build/vortex_utils/ament_cmake_pytest/test_utils.txt" "--append-env" "PYTHONPATH=/home/badawi/Desktop/auto-pilot/build/vortex_utils" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/badawi/Desktop/auto-pilot/src/vortex-utils/tests/test_utils.py" "-o" "cache_dir=/home/badawi/Desktop/auto-pilot/build/vortex_utils/ament_cmake_pytest/test_utils/.cache" "--junit-xml=/home/badawi/Desktop/auto-pilot/build/vortex_utils/test_results/vortex_utils/test_utils.xunit.xml" "--junit-prefix=vortex_utils")
set_tests_properties(test_utils PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/badawi/Desktop/auto-pilot/src/vortex-utils" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/badawi/Desktop/auto-pilot/src/vortex-utils/CMakeLists.txt;44;ament_add_pytest_test;/home/badawi/Desktop/auto-pilot/src/vortex-utils/CMakeLists.txt;0;")
