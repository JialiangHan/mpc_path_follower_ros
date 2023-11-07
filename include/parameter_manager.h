#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace mpc_path_follower
{

  /**
   * @class ParameterManager
   * @brief Config class for the teb_local_planner and its components.
   */
  class ParameterManager
  {
  public:
    std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
    std::string map_frame;  //!< Global planning frame

    double planning_frequency;

    // Parameters
    double max_speed; // max speed that planner should not exceed

    double max_steer_angle;
    double min_speed;
    double vehicle_width;
    double vehicle_length;
    double wheelbase_length;
    double turning_radius;
    double safety_radius;

    bool evaluate_path;
    // speed limit
    float max_linear_velocity;
    float min_linear_velocity;
    float max_angular_acceleration;
    float min_angular_acceleration;
    float max_linear_acceleration;
    float min_linear_acceleration;

    // MPC parameter
    float vehicle_Lf;
    int predicted_length;
    ParameterManager()
    {
      odom_topic = "odom";
      map_frame = "odom";

      planning_frequency = 1;

      // Parameters
      max_speed = 2; // max speed that planner should not exceed

      max_steer_angle = 0.5;
      min_speed = 0.2;

      vehicle_width = 1.25;
      vehicle_length = 1.8;
      wheelbase_length = 1.006;
      turning_radius = 1.5;
      safety_radius = 1.5;

      evaluate_path = false;
      // speed limit
      max_linear_velocity = 5;
      min_linear_velocity = 0;
      max_angular_acceleration = 100;
      min_angular_acceleration = -100;
      max_linear_acceleration = 4;
      min_linear_acceleration = -4;

      vehicle_Lf = 0.325;
      predicted_length = 20;
    }

    /**
     * @brief Load parameters from the ros param server.
     * @param nh const reference to the local ros::NodeHandle
     */
    void loadRosParamFromNodeHandle(const ros::NodeHandle &nh);
  };

} // namespace adaptive_open_local_planner
