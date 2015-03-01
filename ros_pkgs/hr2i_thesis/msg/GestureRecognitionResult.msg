int8 idHello = 0
int8 idNod = 1
int8 idNegate = 2
int8 idPointAt = 3

std_msgs/Header header
int8 gestureId
geometry_msgs/Point ground_point # Only used when gesture is PointAt
geometry_msgs/Point person_position # Position (x, y, z) where the person is
