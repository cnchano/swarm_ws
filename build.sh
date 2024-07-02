# Fix Gazebo on first run bug
source /usr/share/gazebo/setup.bash

# Delete generated folder to make sure no dirty data will be used
rm -r build/ install/ 

# Build & source
colcon build && source install/setup.bash