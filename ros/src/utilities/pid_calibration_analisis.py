import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

file = open("/home/student/Desktop/CarND-Capstone/data/PID_Calibration_data/dbw_node-4.log","r")

target_points = []
current_points = []
steer_PID_REQUEST = []
steer_PID_REQUEST_FILT = []
error = []
error_filt = []

PID_P_vals = []
PID_I_vals = []
PID_D_vals = []

current_speed = []

for l in file:
    body = l.split(": ")[1]
    if(body[0:12] == "SpeedCurrent"):
        current_speed.append(body.split(", ")[0].split("-> ")[1])
        target_points.append(body.split(", ")[3].split("-> ")[1])
        current_points.append(body.split(", ")[2].split("-> ")[1])
    elif(body[0:14] == "Throttle_brake"):
        steer_PID_REQUEST.append(body.split(", ")[4].split("-> ")[1])
        steer_PID_REQUEST_FILT.append(body.split(", ")[3].split("-> ")[1])
        error_filt.append(body.split(", ")[5].split("-> ")[1])
        error.append(body.split(", ")[6].split("-> ")[1])
    else:
        PID_P_vals.append(body.split(", ")[0].split("-> ")[1])
        PID_I_vals.append(body.split(", ")[1].split("-> ")[1])
        PID_D_vals.append(body.split(", ")[2].split("-> ")[1])

plt.figure(1)
plt.subplot(311)

plt.plot(range(len(target_points)), target_points, range(len(target_points)), current_points)
#plt.plot(range(len(steer_PID_REQUEST_FILT)), steer_PID_REQUEST_FILT, range(len(steer_PID_REQUEST_FILT)), steer_PID_REQUEST)

plt.ylabel('Steer Value')
plt.xlabel('Sample')
plt.grid(True)
plt.legend("Target Steer", "PID Steer")

blue_patch = mpatches.Patch(color='blue', label='Target Steer')
green_patch = mpatches.Patch(color='green', label='PID Steer')
plt.legend(handles=[blue_patch,green_patch])

plt.subplot(312)
#plt.plot(range(len(target_points)), current_speed)
plt.plot(range(len(PID_P_vals)), PID_P_vals, range(len(PID_P_vals)), PID_I_vals, range(len(PID_P_vals)), PID_D_vals)

blue_patch = mpatches.Patch(color='blue', label='P val')
green_patch = mpatches.Patch(color='green', label='I val')
red_patch = mpatches.Patch(color='red', label='D val')
plt.legend(handles=[blue_patch,green_patch,red_patch])
plt.grid(True)

plt.subplot(313)
plt.plot(range(len(target_points)), current_speed)
#plt.plot(range(len(error)), error, range(len(error)), error_filt)

plt.ylabel('Speed Value')
plt.xlabel('Sample')
plt.grid(True)

plt.show()