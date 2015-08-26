/*
 * This is a library (world plugin) for gazebo. It will add a vehicle in
 *chrono and use the OnUpdate() loop to run a step(s) of the chrono simulation
 *loop. It will then update the vehicel model pose in gazebo. The goal of this
 *is to have chrono simulate the vehicle and have gazebo output the results as
 *well as simulate sensors and provide feedback in the form of driving
 *parameters into chrono.
 *
 * Created by Asher Elmquist (UW SBEL and OSRF)
*/

#include <vector>
#include <cstdio>
#include <stdio.h>

#include <algorithm>
#include <assert.h>

//chrono vehicle includes
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/vehicle/Vehicle.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

///the chrono-engine includes
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "core/ChRealtimeStep.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "physics/ChLinkDistance.h"
#include "physics/ChLinkMate.h"

///the gazebo includes
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/physics/CylinderShape.hh>
#include <gazebo/physics/SphereShape.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/sensors/sensors.hh>

//gazebo_ros includes
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <std_msgs/Float64.h>

//gazonoVehicle includes
#include "purePursuitDriver.cc"

using namespace chrono;
using namespace gazebo;

class GazonoVehicle : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    //create a world pointer
    this->_world = _parent;
    //disable the physics engine
    _parent->EnablePhysicsEngine(false);

    //BEGIN LINE FOLLOW
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->rosnode_ = new ros::NodeHandle("GazonoVehicle");

    // Custom Callback Queue
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float64>(
      "/track_point",1,
      boost::bind( &GazonoVehicle::UpdateDriverInput,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(so);

    // Custom Callback Queue
    this->callback_queue_thread_ = boost::thread( boost::bind( &GazonoVehicle::QueueThread,this ) );
    //END LINE FOLLOW LOAD

    //CONNECT TO LASER
    this->raySensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>
      (sensors::SensorManager::Instance()->GetSensor(
        this->_world->GetName() + "::vehicle::chassis::laser"));
        // this->world->GetName() + "::" + this->model->GetScopedName()
        // + "::pelvis::"
        // "imu_sensor"));
    if (!this->raySensor)
    std::cout << "laser not found!\n\n";
    //END CONNECT TO LASER

    //create model pointers
    if(_world->GetModel("vehicle") != NULL){
        _model1 = _world->GetModel("vehicle");
    }
    //create vector of wheels for easy reference
    wheels.push_back(_world->GetModel("wheel1"));
    wheels.push_back(_world->GetModel("wheel2"));
    wheels.push_back(_world->GetModel("wheel3"));
    wheels.push_back(_world->GetModel("wheel4"));

    //setup chrono vehicle
    //set initial conditions
    initLoc = (0.0, 0.0, 1.0);
    initRot = ChQuaternion<>(1.0, 0.0, 0.0, 0.0);

    //create the chrono vehicle
    boost::filesystem::path full_path( boost::filesystem::current_path() );

    vehicle = ChSharedPtr<Vehicle>(new Vehicle(vehicle::GetDataFile(vehicle_file)));

    vehicle->Initialize(ChCoordsys<>(initLoc, initRot));

    terrain = ChSharedPtr<RigidTerrain> (new RigidTerrain(vehicle->GetSystem(), terrainHeight, terrainLength, terrainWidth, 0.8));

    powertrain = ChSharedPtr<SimplePowertrain>(new SimplePowertrain(vehicle::GetDataFile(simplepowertrain_file)));

    powertrain->Initialize();

    num_axles = vehicle->GetNumberAxles();
    num_wheels = 2 * num_axles;
    tires = std::vector<ChSharedPtr<RigidTire>>(num_wheels);

    for (int i = 0; i < num_wheels; i++) {
        //create the tires from the tire file
        tires[i] = ChSharedPtr<RigidTire>(new RigidTire(vehicle::GetDataFile(rigidtire_file), *terrain));
        tires[i]->Initialize(vehicle->GetWheelBody(i));
    }
    //create the driver if not using line following with camera
    //driver = ChSharedPtr<ChDataDriver>(new ChDataDriver(vehicle::GetDataFile(driver_file)));
    //pursuitDriver = std::unique_ptr<PurePursuitDriver>(new PurePursuitDriver(10.0));
    //lineDriver = std::unique_ptr<LineFollow>(new LineFollow(10.0));

    tire_forces = ChTireForces(num_wheels);
    wheel_states = ChWheelStates(num_wheels);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
         boost::bind(&GazonoVehicle::OnUpdate, this));

  }
  public: void OnUpdate()
   {
     //control vehicle speed control
     //this will maintain an approximate speed of 7 m/s
     if(vehicle->GetVehicleSpeedCOM() <= 5.0){
       braking_input = 0.0;
       throttle_input = 0.3;
     }
     else if(vehicle->GetVehicleSpeedCOM() >= 7.0){
       braking_input = 0.3;
       throttle_input = 0.1;
     }
     else{
       braking_input = 0.15;
       throttle_input = 0.2;
     }

     //connect to the ray data to stop vehicle in case of obstruction
     raySensor->GetRanges(ranges);
     for(int i=0; i<ranges.size(); i++){
       if(ranges[i] <=100){
         //std::cout<<"range was: "<<ranges[i]<<std::endl;
         //set throttle and brake if object seen
         throttle_input = 0.0;
         braking_input = 0.5;
         //cut out of loop
         i = ranges.size();
       }
     }

     for(int i =0; i<1; i++){
       //collect module information

       //THIS IS WHERE COLLECT DRIVER INFO IF NOT USING LINE FOLLOW WITH CAM
       //throttle_input = driver->GetThrottle();
       //steering_input = driver->GetSteering();
       //braking_input = driver->GetBraking();

       //throttle_input = pursuitDriver->GetThrottle();
       //steering_input = pursuitDriver->GetSteering();
       //braking_input = pursuitDriver->GetBraking();

       //throttle_input = lineDriver->GetThrottle();
       //steering_input = lineDriver->GetSteering();
       //braking_input = lineDriver->GetBraking();

       powertrain_torque = powertrain->GetOutputTorque();
       driveshaft_speed = vehicle->GetDriveshaftSpeed();
       for (int i = 0; i < num_wheels; i++) {
          tire_forces[i] = tires[i]->GetTireForce();
          wheel_states[i] = vehicle->GetWheelState(i);
       }

       //Update modules (process inputs from other modules)
       time = vehicle->GetSystem()->GetChTime();
       //driver->Update(time);
       //pursuitDriver->Update(vehicle);
       powertrain->Update(time, throttle_input, driveshaft_speed);
       vehicle->Update(time, steering_input, braking_input, powertrain_torque, tire_forces);
       terrain->Update(time);
       for (int i = 0; i < num_wheels; i++)
          tires[i]->Update(time, wheel_states[i]);

       // Advance simulation for one timestep for all modules
       //driver->Advance(step_size);
       powertrain->Advance(step_size);
       vehicle->Advance(step_size);
       terrain->Advance(step_size);
       for (int i = 0; i < num_wheels; i++)
          tires[i]->Advance(step_size);
       }

       //update the gazebo vehicle and wheel pose
       _model1->SetWorldPose(math::Pose(
           math::Vector3(
               vehicle->GetChassisPos().x,
               vehicle->GetChassisPos().y,
               vehicle->GetChassisPos().z
           ),
           math::Quaternion(
               vehicle->GetChassisRot().e0,
               vehicle->GetChassisRot().e1,
               vehicle->GetChassisRot().e2,
               vehicle->GetChassisRot().e3
           )), "link");
        for(int j=0; j<wheels.size();j++){
          wheels[j]->SetWorldPose(math::Pose(
              math::Vector3(
                  vehicle->GetWheelPos(j).x,
                  vehicle->GetWheelPos(j).y,
                  vehicle->GetWheelPos(j).z
              ),
              math::Quaternion(
                  vehicle->GetWheelRot(j).e0,
                  vehicle->GetWheelRot(j).e1,
                  vehicle->GetWheelRot(j).e2,
                  vehicle->GetWheelRot(j).e3
              )), "link");
        }
   }

   void UpdateDriverInput(const std_msgs::Float64::ConstPtr& _msg)
    {
      this->steering_input = _msg->data;
    }

    // custom callback queue thread
    void QueueThread()
    {
      static const double timeout = 0.01;

      while (this->rosnode_->ok())
      {
        this->queue_.callAvailable(ros::WallDuration(timeout));
      }
    }
    ~GazonoVehicle(){
      // Custom Callback Queue
      this->queue_.clear();
      this->queue_.disable();
      this->rosnode_->shutdown();
      this->callback_queue_thread_.join();
      delete this->rosnode_;
    }

private: ros::NodeHandle* rosnode_;
    ros::Subscriber sub_;
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;

    //Chrono optional vehicle setups
    ////******THESE ARE BEING PULLED FROM THE DATA DIRECTORY AT
    ////****** Chrono-Gazebo/gazono/data
    // JSON file for vehicle model
    //std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
    //std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle_simple_lugged.json");
    std::string vehicle_file = "hmmwv/vehicle/HMMWV_Vehicle_4WD.json";
    //std::string vehicle_file = "generic/vehicle/Vehicle_DoubleWishbones_ARB.json";
    //std::string vehicle_file("generic/vehicle/Vehicle_DoubleWishbones_ARB.json");
    //std::string vehicle_file("generic/vehicle/Vehicle_MultiLinks.json");
    //std::string vehicle_file("generic/vehicle/Vehicle_SolidAxles.json");
    //std::string vehicle_file("generic/vehicle/Vehicle_ThreeAxles.json");
    //std::string vehicle_file("generic/vehicle_multisteer/Vehicle_DualFront_Independent.json");
    //std::string vehicle_file("generic/vehicle_multisteer/Vehicle_DualFront_Shared.json");

    // JSON files for tire models (rigid) and powertrain (simple)
    std::string rigidtire_file = "generic/tire/RigidTire.json";
    std::string simplepowertrain_file = "generic/powertrain/SimplePowertrain.json";

    // Driver input file (if not using Irrlicht)
    std::string driver_file = "generic/driver/Sample_Maneuver.txt";

    // Initial vehicle location and orientation
    ChVector<> initLoc;
    ChQuaternion<> initRot;

    //ground plane parameters
    double terrainHeight = 0.0;
    double terrainLength = 1000.0;   // size in X direction
    double terrainWidth  = 1000.0;   // size in Y direction

    //chrono vehicle components
    ChSharedPtr<Vehicle> vehicle;
    ChSharedPtr<RigidTerrain> terrain;
    ChSharedPtr<SimplePowertrain> powertrain;
    std::vector<ChSharedPtr<RigidTire> > tires;
    //ChSharedPtr<ChDataDriver> driver;
    std::unique_ptr<PurePursuitDriver> pursuitDriver; //*****************
    std::vector<double> ranges;
    ChTireForces   tire_forces;
    ChWheelStates  wheel_states;

    //chrono vehicle parameters, values should not matter unless no driver
    double         driveshaft_speed;
    double         powertrain_torque;
    double         throttle_input = 0.2;
    double         steering_input = 0.0;
    double         braking_input = 0.1;
    int num_axles;
    int num_wheels;

    //define the step size
    double step_size = 0.001;
    double time = 0.0;

    //pointers to reference the models shared by chrono and gazebo
    boost::shared_ptr<sensors::RaySensor> raySensor;
    std::vector<ChSharedPtr<ChBody>> chronoModels;
    std::vector<physics::ModelPtr> wheels;
    physics::WorldPtr _world;
    physics::ModelPtr _model1;
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazonoVehicle)
