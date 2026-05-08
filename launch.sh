source install/setup.bash

ros2 launch vicon_odometry_estimator client.launch.py \
  hostname:=10.157.106.226                            \
  topic_namespace:=vicon                              \
  buffer_size:=200                                    \
  world_frame:=map                                    \
  vicon_frame:=vicon                                  \
  map_xyz:='[0.0, 0.0, 0.0]'                          \
  map_rpy:='[0.0, 0.0, 0.0]'                          \
  map_rpy_in_degrees:=false                           \
  cutoff_hz:=30.0                                     \
  debug:=false