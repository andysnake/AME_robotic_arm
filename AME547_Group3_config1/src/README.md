# compile files
# First step
open a terminal and type following commands:

mkdir -p catkin_ws

cd catkin_ws

Then put the src folder into the catkin_ws folder

catkin_make

source deverl/setup.bash

roslaunch process_visualizer start_simulation.launch

#Second Step

Then open another terminal and type commands:

cd catkin_ws

source devel/setup.bash

roslaunch ur_planning ur_planning.launch


