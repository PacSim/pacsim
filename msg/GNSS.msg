std_msgs/Header header

# for unknown entries, set -1 on main diagonal

int8 STATUS_NO_FIX =  -1
int8 STATUS_FIX =      0
int8 status

# Latitude [degrees]
float64 latitude
# Longitude [degrees]
float64 longitude
# Altitude [m]. Positive is above the WGS 84 ellipsoid
float64 altitude
# covariance in ENU coordinates
float64[9] position_covariance

# relative to ENU plane
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance

float64 velocity_east
float64 velocity_north
float64 velocity_up
float64[9] velocity_covariance