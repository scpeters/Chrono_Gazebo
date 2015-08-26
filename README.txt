This is a Chrono-Gazebo simulation of a vehicle. The goal is to have this be an
autonomous vehicle using sensors from gazebo and vehicle dynamics from chrono_Vehicle.

To run the vehicle using autonomous drive and ROS:

  simply run the launch script located in Chrono_Gazebo/gazonoVehicle

  OR

  in one terminal:
  $roscore

  in second terminal from inside gazonoVehicle/OpenCV directory:
  $python follower_opencv.py

  in third terminal from inside gazonoVehicle directory:
  $rosrun gazebo_ros gazebo gazonoVehicle.world
  (above, gazebo can be switched for gzserver to run without client: much faster)


To run the vehicle without ROS: (will need code tweaking to work but easy)
  run:
  $gazebo gazonoVehicle.world
  from inside the gazonoVehicle directory


Dependencies:
  ChronoEngine with module Vehicle enabled
  Gazebo without Bullet (currently only tested with gazebo 6.1)
  Ros (currently only tested with indigo)
  Gazebo_ros_packages (currently only tested with ros_indigo)

Strange issues:
  Every now and then, gazebo will crash upon startup. Rerunning should solve issue

If errors occur, likely causes are not having correct files sourced,
incorrect location of data, ...
