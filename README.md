# Command velocity filter (cmd_vel_filter)

The filter subscribes to a twist topic (argument `sub_topic` with default *cmd_vel*) and publishes it to a CARLA twist topic (argument `pub_topic` with default *carla/ego_vehicle/control/set_target_velocity*). 

The angular velocity gets **inverted** as CARLA defines it differently. Also, if all the velocities in the twist message equal zero, the filter sends a **brake msg** to CarlaEgoVehicleControl topic */carla/ego_vehicle/vehicle_control_cmd*. This is to prevent the agent from drifting when the target velocity is 0.