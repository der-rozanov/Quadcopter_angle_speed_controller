# Quadcopter Angular Speed Sontroller

This model describes a quadrocopter angular velocity control system.
The first plot describe the values of the angular speed by the Roll, Pitch and Yaw.
The second plot describe the values of the RPM of each motor

![Screenshot](Program.png)

## Program rewiew

### Lets start from math modelling:

The quadrocopter in space is controlled by the different throttle of each motor. Throttle force produced moment, that call quadcopter rotation 
around X Y and Z axis. In python it seems like this:

        M0 = ((w[0] * pow(b, 2)) * length / 2)  # moment of first motor
        M1 = ((w[1] * pow(b, 2)) * length / 2)  # moment of second motor    
        M2 = ((w[2] * pow(b, 2)) * length / 2)  # moment of third motor
        M3 = ((w[3] * pow(b, 2)) * length / 2)  # moment of fourth motor
        Mz = d * (pow(w[3], 2) + pow(w[1], 2) - pow(w[0], 2) - pow(w[2], 2))  #gyro moment of all motors
        
We are used to counting items starting from one, but motors should be counted from zero. Firs four strings calculates moment of every engine.
different moment on different sides of quadcopter produces rotation around X and Y axis, we call it Pitch and Roll.
However, with the Z axis, everything is more complicated. There are important different values of RPM of motors that rotate clockwise and counterclockwise. If clockwrise rotated motors rotate faster than counterclockwise motors, quadcopter starts rotate counterclockwise. d is special ratio that depends on quadcopter construction.

        Ax = (M0 + M1 - M2 - M3) / inert  # calculate X axis acceleration
        Wx_cur += Ax * dt  # calculate X axis angle speed
        
        Ay = (M0 + M3 - M1 - M2) / inert  # calculate Y axis acceleration
        Wy_cur += Ay * dt  # calculate Y axis angle speed
        
         Az = Mz / inert  # calculate Z axis acceleration
        Wz_cur += Az * dt  # calculate Z axis angle speed
        
Angular acceleration we can find like Moment/inertia moment. Because we write discret system we have sampling frequency that we called dt, so we can
integrate the angular acceleration and get the angular speed like this.  

### Control system 

Okay, but it is difficult for us to control each motor separately. Lets write control system. 

let's say we have some command like "to set the angular acceleration to N, you need do throttle to M" and each expression for Roll and Yaw. Then we need command mixer. Depending on the location of the motors on the frame, we give different commandd for different frame sides.

         w[0] = saturation(P_cmd - Yaw_cmd + Roll_cmd + Pitch_cmd, 20000, 0)  #Top left motor
         w[1] = saturation(P_cmd + Yaw_cmd - Roll_cmd + Pitch_cmd, 20000, 0)  #Top right motor
         w[2] = saturation(P_cmd - Yaw_cmd - Roll_cmd - Pitch_cmd, 20000, 0)  #Bottom right motor
         w[3] = saturation(P_cmd + Yaw_cmd + Roll_cmd - Pitch_cmd, 20000, 0)  #Bottom left motor 

As you can see, to turn the quadcopter by roll clockwrise we need to increase throttle on the left side and decrease on the right side. To turn quadcopter by pitch clockwrise we need to increase throttle on the top side and decrease on the bottom side. To turn quadcopter by yaw we need to increase throttle on the motors, that rotate counterclockwise, and decrease on the mototrs, that rotate clockwrise. In general, it is enough to change the thrust on only one side to turn, but we change the thrust on two sides to speed up the process and ensure the same lifting force. "Sturation" function we need to ensure relistic modelling process - motor cant     
        
    
