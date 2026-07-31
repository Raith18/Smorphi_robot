# ObjectDetection.msg — one 2D detection with fused 3D attributes
string label          # class name (COCO/KITTI subset, e.g. "car")
float32 score         # detection confidence [0,1]
float32[4] bbox       # pixel bbox in image frame: x1 y1 x2 y2
float32 depth         # median fused LiDAR depth of points inside the bbox [m]
float32[3] position   # 3D center in the laser frame [m] (0,0,0 if unavailable)
float32[3] dimensions # 3D box l w h [m] (0,0,0 if unavailable)
