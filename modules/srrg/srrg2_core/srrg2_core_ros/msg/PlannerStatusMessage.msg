#gg mesage for the planner status, shot continuosly by the planner

Header   header
string   status           #status of the planner ("idle", "moving", "invalid_goal", "unreachable")
int32    a_star_status    #result of the enum for the local path search
int32    path_type        #type of path 0: gradient, 1: grid
float32  cost_to_global_goal     #value of the cost function at the current location
float32  distance_to_global_goal #distance to the current location [meters]
float32  distance_to_local_goal #distance to the current location [meters]
