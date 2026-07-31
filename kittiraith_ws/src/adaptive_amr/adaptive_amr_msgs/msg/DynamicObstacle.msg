# DynamicObstacle.msg — one predicted dynamic obstacle
int32 track_id                # matches object_tracking track_id
string label                  # class name (e.g. "car")
float32 score                 # latest track confidence
geometry_msgs/Point position        # current position (laser frame) [m]
geometry_msgs/Vector3 velocity      # velocity estimate [m/s]
geometry_msgs/Point[] predicted_trajectory   # future positions at horizon steps
float64 horizon                # prediction horizon [s]
float32 collision_risk         # heuristic 0..1 (1 = likely collision course)
