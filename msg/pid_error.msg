# used for store PID error

float32[3] pos  #position error
float32[3] pos_p  
float32[3] pos_i 
float32[3] pos_d

float32[3] vel  #velocity error
float32[3] vel_p
float32[3] vel_i
float32[3] vel_d

float32[3] ang  #angular error
float32[3] ang_p
#float32[3] ang_i
#float32[3] ang_d
float32 yaw_f

float32[3] rat  #angular rate error
float32[3] rat_p
float32[3] rat_i
float32[3] rat_d
float32[3] rat_f
