#include "ros/ros.h"
#include "motor_control/coordinatePass.h"
#include "DataHandle.h"

DataHandle::DataHandle(){

}

void DataHandle::requestCoordinate(){

  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<motor_control::coordinatePass>("coordinatePass");
  motor_control::coordinatePass srv;
  srv.request.request = true;
  client.call(srv);
}

DataHandle::~DataHandle(){
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "coordinatePassClient");
  DataHandle client;

  client.requestCoordinate();

  return 0;
}