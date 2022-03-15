import serial
from tkinter import *

ser = serial.Serial('COM19',9600) #COM Port may need to be changed in some cases
ser.close()
ser.open()

##Variable
backgnd = "white"

##Window configuration/setting
window = Tk()
window.title('Regulator coefficients')
window.geometry("1080x720")
window.minsize(720,480)
window.iconbitmap("C:/Users/Arthur/Desktop/coupe de france de robotique/logo.ico") #Should be changed depending on where the image is placed
window.config(background=backgnd)

frameL = Frame(window, bg = backgnd, bd = 2)
frameR = Frame(window, bg = backgnd, bd = 2)

##Create and position elements
lines = []
mini = 0

def line_regulation(parent,name,min,max,resol):
    line_frame = Frame(parent, bg = backgnd)
    txt = Label(line_frame,text=name, font = ("Arial",10), bg = backgnd, fg="black")
    spinbox_left = Spinbox(line_frame,from_=min,to=max, increment = resol,bg=backgnd,width=10)
    return [parent, txt, spinbox_left, line_frame]


def show_line(line):
    line[1].pack(side = LEFT)
    line[2].pack(side = LEFT)
    line[3].pack()
    return

def send_values():
    message = 's'
    for l in lines:
        message += str(l[2].get())
        message += ';'
    message += 'e'
    ser.write(str.encode(message))
    print(message)
    return


#Length part
title_length = Label(frameL,text="Length coefficients", font = ("Arial",20), bg = backgnd, fg="black")
title_length.pack()

lineKpl = line_regulation(frameL, "\nKp   \n",0.01,100,0.01)
lineKil = line_regulation(frameL, "\nKi   \n",0,1,0.001)
lineKdl = line_regulation(frameL, "\nKd   \n",0,1,0.001)
lines = [lineKpl,lineKil,lineKdl]
for l in lines:
    show_line(l)

#Angle part
title_angle = Label(frameL,text="\nAngle coefficients", font = ("Arial",20), bg = backgnd, fg="black")
title_angle.pack()

lineKpa = line_regulation(frameL, "\nKp   \n",1,100,1)
lineKia = line_regulation(frameL, "\nKi   \n",0.001,1,0.001)
lineKda = line_regulation(frameL, "\nKd   \n",0.001,1,0.001)


lines = [lineKpl,lineKil,lineKdl,lineKpa,lineKia,lineKda]
for i in range(3,6):
    show_line(lines[i])

#send buttun
spacer1 = Label(frameL, text="",bg = backgnd)
spacer1.pack()
btn_send = Button(frameL,text="Send values to Arduino",font = ("Arial",10), bd=5, padx=5,pady=5,command=send_values)
btn_send.pack()

#Left side pack
frameL.pack(side = LEFT)

##Command section

##Launch window
window.mainloop()
ser.close()