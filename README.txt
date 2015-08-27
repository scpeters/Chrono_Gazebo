This is a Chrono-Gazebo simulation of a vehicle. This simulates an autonomous vehicle using sensors from gazebo and vehicle dynamics from chrono::Vehicle.

INSTALL:
  Install ROS (only indigo tested) from deb or source
    Instructions: http://wiki.ros.org/ROS/Installation
  
  Install gazebo_ros packages from source
    Instructions: http://gazebosim.org/tutorials?tut=ros_installing&ver=1.9%2B&cat=connect_ros
  
  Install Gazebo from source - DO NOT install bullet (causes issues for now)
    Instructions: http://gazebosim.org/tutorials?tut=install_from_source&ver=default&cat=install
  
  Install Chrono
    Instructions:
      clone https://github.com/projectchrono/chrono
      using cmake: 
        source should be the top directory
        build directory can be wherever
        enable module_vehicle 
      make; make install

BUILD:
  clone this repository
  create build directory inside gazonoVehicle - this will give correct path for vehicle data
  cmake ..; make

RUN:
  $./launch from inside gazonoVehicle directory
  
  Can modify launch script to run desired configuration
  

NOTES:

CURRENTLY TESTED ON UBUNTU TRUSTY 14.04
Build mode for Chrono and Gazebo should be RELEASE for speed
If run with gzserver instead of gazebo, will run faster but only camera for visual

To run the vehicle without ROS: (will need code tweaking to work but easy)
  run:
  $gazebo gazonoVehicle.world
  from inside the gazonoVehicle directory

Strange issues:
  Every now and then, gazebo will crash upon startup. Rerunning should solve issue

If errors occur, likely causes are not having correct files sourced,
incorrect location of data, ...
