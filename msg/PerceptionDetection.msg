std_msgs/Header header
geometry_msgs/PoseWithCovariance pose

uint8 CLASS_UNKNOWN = 0
uint8 CLASS_INVISIBLE = 1
uint8 CLASS_BLUE = 2
uint8 CLASS_YELLOW = 3
uint8 CLASS_ORANGE = 4
uint8 CLASS_BIGORANGE = 5
uint8 CLASS_TIMEKEEPING = 6

float64[7] class_probabilities # highest probability -> "detection class" 
float64 detection_probability # 1.0 -> for sure true positive : maybe false positive