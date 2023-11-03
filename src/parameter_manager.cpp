

#include "parameter_manager.h"

namespace mpc_path_follower
{

  void ParameterManager::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
  {

    nh.getParam("odom_topic", odom_topic);
    nh.getParam("map_frame", map_frame);

    nh.getParam("max_speed", max_speed);

    nh.getParam("max_steer_angle", max_steer_angle);
    nh.getParam("min_speed", min_speed);

    nh.getParam("vehicle_width", vehicle_width);
    nh.getParam("vehicle_length", vehicle_length);
    nh.getParam("wheelbase_length", wheelbase_length);
    nh.getParam("turning_radius", turning_radius);
    nh.getParam("safety_radius", safety_radius);

    nh.getParam("evaluate_path", evaluate_path);

    nh.getParam("max_linear_velocity", max_linear_velocity);
    nh.getParam("min_linear_velocity", min_linear_velocity);
    nh.getParam("max_angular_acceleration", max_angular_acceleration);
    nh.getParam("min_angular_acceleration", min_angular_acceleration);
    nh.getParam("max_linear_acceleration", max_linear_acceleration);
    nh.getParam("min_linear_acceleration", min_linear_acceleration);

    nh.getParam("Lf", Lf);
    nh.getParam("predicted_length", predicted_length);
  }

} // namespace adaptive_open_local_planner
