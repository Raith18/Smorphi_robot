# ObjectTrack.msg — one tracked object with a stable identity
int32 track_id        # stable ID while the object is tracked
string label          # class name
float32 score         # latest detection confidence
float32[4] bbox       # pixel bbox x1 y1 x2 y2
geometry_msgs/Point position    # 3D position in the laser frame [m]
geometry_msgs/Vector3 velocity  # velocity estimate [m/s]
int32 age             # frames since the track was created
int32 hit_streak      # consecutive detections matched to this track
