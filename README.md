# walker_algorithm_robot

### Building the ROS package
```bash
# Source to ros humble
source /opt/ros/humble/setup.bash
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws/src
git clone https://github.com/Ishaan1810/walker_algorithm_robot.git

# Once files are checked, go back to the root directory of ros workspace
cd ..
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select turtlebot3_walker_ros2
# After successfull build source the package
. install/setup.bash
```


### Using the launch file

**Run below commands to check the functionality of the Rosbag while the turtlebot is running**
```bash
cd ~/ros2_ws/src/walker_algorithm_robot/results/bag_recording
# Using launch file to run publisher to launch the turtlebot on twist topic
ros2 launch walker_algorithm_robot custom_launch.py bag_record:=True

# Press Ctrl+C after 15-20 seconds

```
  
### CppCheck & CppLint
```bash
# Use the below command for cpp check by moving to directory beginner_tutorials
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) --check-config  &> results/cppcheck.txt

# Use the below command for cpp lint by moving to directory beginner_tutorials 
cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint.txt 

## The results of both are present in results folder inside results directory
```
