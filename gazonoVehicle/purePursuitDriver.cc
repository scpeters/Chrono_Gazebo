/**
* This is a simple implementation of a pure pursuit driver. It is intended to
*calculate the steering, braking, and throttle values needed to follow a certain
* path and return those values as inputs into the gazono vehicle.
*
*    Created by Asher Elmquist (UW SBEL and OSRF)
*/

#include "chrono_vehicle/vehicle/Vehicle.h"
#include "core/ChVector.h"
#include "gazebo/gazebo.hh"



class PurePursuitDriver{
public:
  PurePursuitDriver(double look){
    steerValue = 0.0;
    throttleValue = 0.2;
    brakeValue = 0.1;
    lookAhead = look;
    trackT = look/1.7;
    PI = 2*acos(0);
  }
  void Update(chrono::ChSharedPtr<chrono::Vehicle> vehicle){
    //get and save the vehicle position
    vehiclePos = vehicle->GetChassisPosCOM();

    //calculate the current path position
    //should be interestion of line norm through r(t1) and vehicle pos
    //pathT = vehiclePos.x;
    //pathPos.x = pathT;
    //pathPos.y = 10;

    //estimate the current path position
    lastT = pathT;
    guessT = pathT;
    currDist = (pathT-vehiclePos.x)*(pathT-vehiclePos.x) +
        (25*sin((PI*pathT)/50.0)-vehiclePos.y)*(25*sin((PI*pathT)/50.0)-vehiclePos.y);
    double tempT = pathT;
    while((guessT-tempT)<0.1){
      guessT+=step;
      if(((guessT-vehiclePos.x)*(guessT-vehiclePos.x) +
          (25*sin((PI*guessT)/50.0)-vehiclePos.y)*(25*sin((PI*guessT)/50.0)-vehiclePos.y))
          <currDist){
        pathT = guessT;
      }
      if((pathT - (tempT+.1)) <= (step)){
        tempT = pathT;
      }
    }
    pathPos.x = pathT;
    pathPos.y = 25*sin((PI*pathT)/50.0);


    //calculate the track position
    //this should be finding t2 at distance lookAhead from t1
    // from integral from t1 to t2 of mag of velocity
    // trackT = lookAhead + pathT;
    // trackPos.x = trackT;
    // trackPos.y = 10;

    //use a similar approach to find look ahead point
    //diffT = pathT - lastT;
    //trackT +=diffT;
    //trackPos.x = trackT;
    //trackPos.y = 25*sin((PI*trackT)/50.0);


    lookDist = sqrt((trackT-pathT)*(trackT-pathT) +
        (25*sin((PI*trackT)/50.0)-25*sin((PI*pathT)/50.0))*(25*sin((PI*trackT)/50.0)-25*sin((PI*pathT)/50.0)));
    while (abs(lookDist - lookAhead)>0.1){
      if((lookDist-lookAhead)>0){
        trackT-=step;
      }else if((lookDist-lookAhead)<0){
        trackT+=step;
      }
      lookDist = sqrt((trackT-pathT)*(trackT-pathT) +
          (25*sin((PI*trackT)/50.0)-25*sin((PI*pathT)/50.0))*(25*sin((PI*trackT)/50.0)-25*sin((PI*pathT)/50.0)));
    }
    trackPos.x = trackT;
    trackPos.y = 25*sin((PI*trackT)/50.0);


    //calculate the vector to the track position
    errorVec.x = trackPos.x - vehiclePos.x;
    errorVec.y = trackPos.y - vehiclePos.y;
    errorVec = errorVec.GetNormalized();

    //calculate the vehicle heading vector
    headVec.x = 1.0;
    headVec.y = 0;
    headVec.z = 0;
    headVec = vehicle->GetChassisRotCOM().Rotate(headVec);
    headVec.z = 0;
    headVec = headVec.GetNormalized();

    //calculate the error angle to the track position
    errorAngle = atan2(errorVec.y, errorVec.x) - atan2(headVec.y, headVec.x);

    //set the new steering value based on the error angle
    steerValue = errorAngle / approxMaxSteer;
    if(steerValue < -1.0) steerValue = -1.0;
    else if(steerValue > 1.0) steerValue = 1.0;

    //print out values to see what is going on
    // std::cout<<"heading: "<<headVec.x<<", "<<headVec.y<<", "<<headVec.z<<std::endl;
    // std::cout<<"path point: "<<pathPos.x<<", "<<pathPos.y<<", "<<pathPos.z<<std::endl;
    // std::cout<<"track point: "<<trackPos.x<<", "<<trackPos.y<<", "<<trackPos.z<<std::endl;
    // std::cout<<"vehicle point: "<<vehiclePos.x<<", "<<vehiclePos.y<<", "<<vehiclePos.z<<std::endl;
    // std::cout<<"error angle: "<<errorAngle<<std::endl;
    // std::cout<<"steer: "<<steerValue<<std::endl;
    //std::cout<<"velocity: "<<vehicle->GetVehicleSpeedCOM()<<std::endl;

    //control speed so we dont lose grip
    if(vehicle->GetVehicleSpeedCOM() <= 5.0){
      brakeValue = 0.0;
      throttleValue = 0.3;
    }
    else if(vehicle->GetVehicleSpeedCOM() >= 7.0){
      brakeValue = 0.4;
      throttleValue = 0.1;
    }
    else{
      brakeValue = 0.15;
      throttleValue = 0.2;
    }

  }

  double GetSteering(){
      return steerValue;
  }
  double GetThrottle(){
    return throttleValue;
  }
  double GetBraking(){
    return brakeValue;
  }
  ~PurePursuitDriver(){
    delete this;
  }
private:
  //for output
  double steerValue;
  double throttleValue;
  double brakeValue;

  bool notfound = true;

  //for internal use
  double lookAhead;
  chrono::ChVector<double> vehiclePos = {0,0,0};
  chrono::ChVector<double> trackPos = {0,0,0};
  chrono::ChVector<double> pathPos = {0,0,0};
  chrono::ChVector<double> headVec = {1,0,0};
  chrono::ChVector<double> errorVec = {0,0,0};
  double approxMaxSteer = 0.7;
  double errorAngle = 0.0;
  double yawAngle = 0.0;
  double pathT = 0.0;
  double trackT = 0.0;
  double PI;

  //for finding current point
  double guessT = 0.0;
  double step = 0.001;
  double currDist = 0.0;
  double lastT = 0.0;
  double diffT = 0.0;
  double lookDist = 0.0;
  //double distError = 0.0;



  //temp equation
  //r(t)=<t,10>
  //
  //v(t) = <1,0>
  //
  //norm(t) = <0,1>

  //path position equation:
  //  r(t) = <t, 25sin((pi*t)/50)
  //
  // velocity
  // v(t) = <1, pi/2*cos((pi*t)/50)
  //
  // normal to the tangent
  // norm(t) = <pi/2*cos((pi*t)/50), -1>














  //


};
