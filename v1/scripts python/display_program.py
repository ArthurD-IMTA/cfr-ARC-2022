import serial
import numpy as np
import matplotlib.pyplot as plt

plt.ion(); #interactive mode | real time mode

##Constants
dist_wheels = 252
robot_width = 270
robot_length = 260
from_top_distance = 155+75/2 #distance between the top of the robot and the position point (center of the line between the wheels
hr = 140 #distance to the front wheel

##Setup of each graph
#All figures
fig, axs = plt.subplots(2, 2)
axs[0,1] = plt.subplot(222)
axs[1,1] = plt.subplot(224)
space = plt.subplot(121)


#For wheels
def init_wheels():
    angle = np.linspace(0, 2 * np.pi, 50)
    R=1

    axs[0,1].set_title('Left Wheel orientation')
    axs[0,1].plot(R * np.cos(angle), R * np.sin(angle),color = "black")
    axs[0,1].plot([0,1],[0,0],color = "black")
    axs[0,1].set_aspect('equal','box')


    axs[1,1].set_title('Right Wheel orientation')
    axs[1,1].plot(R * np.cos(angle), R * np.sin(angle),color = "black")
    axs[1,1].plot([0,1],[0,0],color = "black")
    axs[1,1].set_aspect('equal','box')
    return


#For global space
def init_board():
    space.set_title('Table with Robot')
    space.set_xlabel('X axis')
    space.set_ylabel('Y axis')
    space.plot([0,1175,1175,1325,1325,1490,2000,2000,1490,1325,1325,1175,1175,0,0,85,85,0,0,100,100,0,0,300,300,0,0,100,100,0,0,85,85,0,0],[0,0,102,102,0,0,510,3000-510,3000,3000,3000-102,3000-102,3000,3000,2550,2550,1830,1830,1725,1725,1575,1575,1511,1511,1489,1489,1425,1425,1275,1275,1170,1170,450,450,0],linewidth=0.5,linestyle='-',color = "black")
    space.plot([0, 2000],[1500,1500],linewidth=0.5,linestyle='--',color = "blue")
    space.set_aspect('equal','box')
    return


##Print of the robot
points_robots = [
    [-robot_width/2,from_top_distance],
    [-robot_width/2,from_top_distance-robot_length],
    [robot_width/2,from_top_distance-robot_length],
    [robot_width/2,from_top_distance],
    [-robot_width/2,from_top_distance]] #describe the four points that constitute the robot (extremities)

def robot_print(x, y, theta):
    init_board()
    M = np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]]) #rotation matrix
    points=[]
    X,Y = [],[]
    for p in points_robots:
        points.append(np.dot(p,M))
        X.append(points[-1][0]+x)
        Y.append(points[-1][1]+y)

    #plt.subplot(121)
    space.plot(X,Y,color='red',linewidth=0.5)

    #front wheel
    angle = np.linspace(0, 2 * np.pi, 20)
    R=15
    center = [0,hr]
    center = np.dot(center,M)
    space.plot(R * np.cos(angle)+center[0]+x, R * np.sin(angle)+center[1]+y,color = "black")
    return

def wheel_print(number, theta_wheel):
    axs[number,1].plot([0,np.cos(theta_wheel)],[0,np.sin(theta_wheel)],linewidth = 1, color = 'blue', linestyle='--')
    return

##Variables
#initial position
#here, x and y represente the center of the line between wheels
x_start = 700
y_start = 100
theta_start = 0



x_shift=0
y_shift=0
theta_shift=0

##Communication and drawing
ser = serial.Serial('COM19',9600) #COM Port may need to be changed in some cases
ser.close()
ser.open()

i = 0

while (i<200): #Define an end to the program
    #Clear before write
    space.clear()
    axs[0,1].clear()
    axs[1,1].clear()

    #data transmission
    data_received = ser.readline()
    #print("New one : ")
    data_received = data_received.decode()
    #print(data_received +'\n')
    data_received = data_received.split(";")

    init_wheels()
    robot_print(x_start+int(data_received[0]),y_start+int(data_received[1]),theta_start+float(data_received[2]))
    wheel_print(0,float(data_received[4]))
    wheel_print(1,float(data_received[3]))
    plt.show()
    plt.pause(0.5)
    i +=1

ser.close()






