import numpy as np
import pylab

# -------------------------------------
g = 9.81
Timelapse = 1
dt = 0.01

mass = 1
inert = 0.000001
length = 0.15
b = 0.001
d = 0.0000000001

Wx_start = 0
Wy_start = 0
Wz_start = 0

P_des = 50
Wx_des = 0.5
Wy_des = 0.4
Wz_des = 0.1
# -------------------------------------


def saturation(var, max, min):
    if var > max:
        return max
    elif var < min:
        return min
    else:
        return var


def PID(pid_input_parameters):  # [kp,ki,kd, setpoint, currentpoint, old_error, sum_error,dt]

    # variables definition
    kp = pid_input_parameters[0]
    ki = pid_input_parameters[1]
    kd = pid_input_parameters[2]
    setpoint = pid_input_parameters[3]
    currentpoint = pid_input_parameters[4]
    old_error = pid_input_parameters[5]
    sum_error = pid_input_parameters[6]
    dt = pid_input_parameters[7]

    # calculate PID impact
    error = setpoint - currentpoint
    impact = kp * error + kd * ((error - old_error) / dt) + ki * sum_error

    return impact


def Mixer(P_cmd, Roll_cmd, Pitch_cmd, Yaw_cmd, w):
    # mixing commands for quadcopter
    w[0] = saturation(P_cmd - Yaw_cmd + Roll_cmd + Pitch_cmd, 10000, 0)
    w[1] = saturation(P_cmd + Yaw_cmd - Roll_cmd + Pitch_cmd, 10000, 0)
    w[2] = saturation(P_cmd - Yaw_cmd - Roll_cmd - Pitch_cmd, 10000, 0)
    w[3] = saturation(P_cmd + Yaw_cmd + Roll_cmd - Pitch_cmd, 10000, 0)


def PhysModel(P_des, Wx_des, Wy_des, Wz_des, Timelapse, dt):
    w = [0, 0, 0, 0]
    Motors_data = [[], [], [], []]
    time = 0.0

    Wx = []
    Wx_cur = Wx_start
    Wx_old_error = 0
    Wx_sum_error = 0

    Wy = []
    Wy_cur = Wy_start
    Wy_old_error = 0
    Wy_sum_error = 0

    Wz = []
    Wz_cur = Wz_start
    Wz_old_error = 0
    Wz_sum_error = 0

    while time < Timelapse:
        Wx_input_parameters = [100, 0.001, 0.1, Wx_des, Wx_cur, Wx_old_error, Wx_sum_error, dt]
        Wx_impact = PID(Wx_input_parameters)  # calculate X axis impact
        Pitch_cmd = saturation(Wx_impact, 500, 0)

        Wy_input_parameters = [100, 0.001, 0.1, Wy_des, Wy_cur, Wy_old_error, Wy_sum_error, dt]
        Wy_impact = PID(Wy_input_parameters)  # calculate Y axis impact
        Roll_cmd = saturation(Wy_impact, 500, 0)

        Wz_input_parameters = [500, 1, 1, Wz_des, Wz_cur, Wz_old_error, Wz_sum_error, dt]
        Wz_impact = PID(Wz_input_parameters)  # calculate Z axis impact
        Yaw_cmd = saturation(Wz_impact, 10, -10)

        Mixer(P_des, Roll_cmd, Pitch_cmd, Yaw_cmd, w)  # do command mix

        M0 = ((w[0] * pow(b, 2)) * length / 2)  # calculate moment of every engine
        M1 = ((w[1] * pow(b, 2)) * length / 2)
        M2 = ((w[2] * pow(b, 2)) * length / 2)
        M3 = ((w[3] * pow(b, 2)) * length / 2)
        Mz = d * (pow(w[3], 2) + pow(w[1], 2) - pow(w[0], 2) - pow(w[2], 2))  # calculate gyro moment

        Ax = (M0 + M1 - M2 - M3) / inert  # calculate X axis acceleration
        Wx_cur += Ax * dt  # calculate X axis angle speed
        Wx_sum_error += (Wx_des - Wx_cur)  # calculate error and remember old error (need for PID)
        Wx_old_error = Wx_des - Wx_cur

        Ay = (M0 + M3 - M1 - M2) / inert  # calculate Y axis acceleration
        Wy_cur += Ay * dt  # calculate Y axis angle speed
        Wy_sum_error += (Wy_des - Wy_cur)  # calculate error and remember old error (need for PID)
        Wy_old_error = Wy_des - Wy_cur

        Az = Mz / inert  # calculate Z axis acceleration
        Wz_cur += Az * dt  # calculate Z axis angle speed
        Wz_sum_error += (Wz_des - Wz_cur)  # calculate error and remember old error (need for PID)
        Wz_old_error = Wz_des - Wz_cur

        time += dt  # next time step

        Motors_data[0].append(w[0] * 100)  # write motors data
        Motors_data[1].append(w[1] * 100)
        Motors_data[2].append(w[2] * 100)
        Motors_data[3].append(w[3] * 100)

        Wx.append(Wx_cur * 57.3)  # write angels speed data
        Wy.append(Wy_cur * 57.3)
        Wz.append(Wz_cur * 57.3)

    return [Wx, Wy, Wz, Motors_data]  # this is proc output, that we will draw or analyse


if __name__ == '__main__':
    arr = PhysModel(P_des,Wx_des,Wy_des,Wz_des, Timelapse, dt)

    Wx = np.array(arr[0])
    Wy = np.array(arr[1])
    Wz = np.array(arr[2])

    Motor1 = np.array(arr[3][0])
    Motor2 = np.array(arr[3][1])
    Motor3 = np.array(arr[3][2])
    Motor4 = np.array(arr[3][3])

    hrz = np.arange(0, Timelapse, dt)

    pylab.figure(figsize=(7, 7), num='Angle speed controller')

    pylab.subplot(211)
    pylab.title("Angle speed", fontsize=12)
    pylab.xlabel("Time, sec", color="black")
    pylab.ylabel("Angle speed, deg/sec ", color="black")
    pylab.plot(hrz, Wx, hrz, Wy, hrz, Wz)
    pylab.legend(["Pitch", "Roll", "Yaw"], loc=1)

    pylab.subplot(212)
    pylab.title("Motors RPM", fontsize=12)
    pylab.xlabel("Time, sec")
    pylab.ylabel("RPM")
    pylab.plot(hrz, Motor1, hrz, Motor2, hrz, Motor3, hrz, Motor4)
    pylab.legend(["Motor 1", "Motor 2", "Motor 3", "Motor 4"], loc=1)

    pylab.tight_layout()
    pylab.show()
