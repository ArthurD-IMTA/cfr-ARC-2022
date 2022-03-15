import matplotlib.pyplot as plt
import numpy as np

fig, axs = plt.subplots(2, 1)
axs[0] = plt.subplot(211)
axs[1] = plt.subplot(212)


pi = 3.14159265358979323

def modulo(x,y):
  return x - int(x/y)*y

def sign(x):
  if (x>0):
    return 1
  if (x<0):
    return -1
  return 0

def setpoint_generation(amax, vmax, vi,  qi,  qf, t):
#t must be greater than 0
  if (qi == qf):
    return qi

  amax = sign(qf-qi)*amax
  vmax = sign(qf-qi)*vmax

  tau = abs((vmax-vi)/amax)
  T = (qf+amax*tau*tau)/vmax-tau
  if (T>tau):
    if (t<=tau):
      return qi+vi*t+0.5*amax*t*t

    elif (t>tau and t<=T):
      return vmax*t-0.5*amax*tau*tau

    elif (t>T and t<= T+tau):
      return vmax*t-0.5*amax*((t-T)*(t-T)+tau*tau)

    else :
      return qf

  elif (T<=tau):
    tau2 = (-vi+(vi*vi+amax*(qf-qi))**0.5)/amax

    if(tau2<0):
      tau2 = (-vi-(vi*vi+amax*(qf-qi))**0.5)/amax

    if (t<=tau2):
      return vi*t+0.5*amax*t*t+qi

    elif (t>tau2 and t<=2*tau2):
      return vi*t+amax*(tau2*tau2-0.5*(2*tau2-t)*(2*tau2-t))+qi
    else :
      return qf


def angle_setpoint( theta,  theta_setpoint):
  theta_setpoint = modulo(theta_setpoint,2*3.1415)
  theta_mod = modulo(theta,2*3.1415)

  if (abs(theta_setpoint-theta_mod)<=3.1415):
    return theta+(theta_setpoint-theta_mod)

  elif ((theta_mod-theta_setpoint)>3.1415):
    return theta+2*3.1415-theta_mod+theta_setpoint

  else :
    return theta-2*3.1415-theta_mod+theta_setpoint


def length_setpoint( xi,  yi,  xf,  yf):
  return ((xf-xi)*(xf-xi)+(yf-yi)*(yf-yi))**0.5

def follow_orientation(xi,yi,xf,yf):
  if ((xf-xi)>0):
    return np.arctan((yf-yi)/(xf-xi))-pi/2
  elif((xf-xi)<0):
    return pi/2+np.arctan((yf-yi)/(xf-xi))
  return sign(yf-yi)*pi/2


def tree_phases_trajectory( xi,  yi,  thetai,  xf,  yf,  thetaf):
    lc = length_setpoint(xi, yi, xf, yf)
    #theta_forward =
    thetac1 = angle_setpoint(thetai,)
    return

def rad(angle):
    return 2*pi*angle/360

def deg(angle):
    return 360*angle/(2*pi)



##Display
xi, yi, thetai = 3, 2, rad(120)
xf, yf, thetaf = 230, 250, rad(233)

amax = 200 #mm/s²
vmax = 1000 #mm/s
vi = 0 #mm/s

amax_rot = 0.2 #rad/s²
vmax_rot = 2 #rad/s
vi_rot = 0 #rad/s

L = []
theta = []
time = []


def plot_trajectory():
  end = 0
  t=0

  while (end==0):
    next = setpoint_generation(amax_rot, vmax_rot, vi_rot, thetai, angle_setpoint(thetai,follow_orientation(xi,yi,xf,yf)), t)
    theta.append(next)
    L.append(0)
    if (next == angle_setpoint(thetai,follow_orientation(xi,yi,xf,yf))):
      end = 1
    time.append(t)
    t += 0.01

  t_end = t
  end = 0

  while (end==0):
    next = setpoint_generation(amax, vmax, vi, 0, length_setpoint(xi,yi,xf,yf), t-t_end)
    L.append(next)
    theta.append(theta[-1])
    if (next == length_setpoint(xi,yi,xf,yf)):
      end = 1
    time.append(t)
    t += 0.01

  t_end = t
  end = 0

  while (end==0):
    next = setpoint_generation(amax_rot, vmax_rot, vi_rot, angle_setpoint(thetai,follow_orientation(xi,yi,xf,yf)), thetaf, t-t_end)
    theta.append(next)
    L.append(L[-1])
    if (next == thetaf):
      end = 1
    time.append(t)
    t += 0.01

  axs[0].plot(time,theta)
  axs[1].plot(time,L)
  return

plot_trajectory()
plt.show()
