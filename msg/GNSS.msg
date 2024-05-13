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
float64[9] position_covariance

geometry_msgs/Quaternion heading
float64[9] heading_covariance

float64 velocity_east
float64 velocity_north
float64 velocity_up
float64[9] velocity_covariance