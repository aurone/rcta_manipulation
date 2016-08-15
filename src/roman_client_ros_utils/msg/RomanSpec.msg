# this timestamp is for debug purposes
uint64 utime

uint8 num_mechanisms
bool[] valid

# torso and limb waypoints (braked position controlled mechanisms) 
uint16 num_waypoints
# waypoints contain their own relative utime 
RomanWaypoint[] waypoints
# dont use more that 1024 waypoints in a spec.
uint16 max_waypoints=1024


##TODO: hand functionality
#uint16 num_hand1_waypoints
#RobotiqSimpleCmd[] hand1_waypoints
#uint16 num_hand2_waypoints
#RobotiqSimpleCmd[] hand2_waypoints

##TODO: tracks functionality
#uint16 num_tracks_waypoints
#TalonCmd[] track_waypoints
