# ARGoS3 ROS Physics Plug-In

This project is a physics-engine for argos3 that allows physics data to be taken from ROS.

## How to install

If you have a local installation of ARGoS3 in $ARGOS_INSTALL_PATH/argos3-dist:

catkin_make -DCMAKE_INSTALL_PREFIX=$ARGOS_INSTALL_PATH/argos3-dist -DCMAKE_BUILD_TYPE=Release install

## How to use

Add the following entry to the physics_engine of your .argos file:
<code><iridia_tracking_system id="its" translate_x="0" translate_y="0" rotate_phi="0" topic="odometry/filtered"/></code>
