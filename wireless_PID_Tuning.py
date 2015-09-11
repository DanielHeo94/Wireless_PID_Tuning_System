from Tkinter import *
import serial
import string

update = serial.Serial('/dev/ttyAMA0', 9600)


class App:
	
    def __init__(self, master):
        frame = Frame(master)
        frame.pack()
	
	Label(frame, text='PITCH_GAIN').grid(row=0, column=0)
        Label(frame, text='P_GAIN').grid(row=1, column=1)
        Label(frame, text='I_GAIN').grid(row=2, column=1)
        Label(frame, text='D_GAIN').grid(row=3, column=1)

	Label(frame, text='ROLL_GAIN').grid(row=4, column=0)
	Label(frame, text='P_GAIN').grid(row=5, column=1)
        Label(frame, text='I_GAIN').grid(row=6, column=1)
        Label(frame, text='D_GAIN').grid(row=7, column=1)

	Label(frame, text='YAW_GAIN').grid(row=8, column=0)
	Label(frame, text='P_GAIN').grid(row=9, column=1)
        Label(frame, text='I_GAIN').grid(row=10, column=1)
        Label(frame, text='D_GAIN').grid(row=11, column=1)

        scalePPGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updatePPGAIN)
        scalePPGAIN.grid(row=1, column=2)
        scalePIGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updatePIGAIN)
        scalePIGAIN.grid(row=2, column=2)
        scalePDGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updatePDGAIN)
        scalePDGAIN.grid(row=3, column=2)

	scaleRPGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updateRPGAIN)
        scaleRPGAIN.grid(row=5, column=2)
        scaleRIGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updateRIGAIN)
        scaleRIGAIN.grid(row=6, column=2)
        scaleRDGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updateRDGAIN)
        scaleRDGAIN.grid(row=7, column=2)

	scaleYPGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updateYPGAIN)
        scaleYPGAIN.grid(row=9, column=2)
        scaleYIGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updateYIGAIN)
        scaleYIGAIN.grid(row=10, column=2)
        scaleYDGAIN = Scale(frame, from_=30, to=30000,
              orient=HORIZONTAL, command=self.updateYDGAIN)
        scaleYDGAIN.grid(row=11, column=2)


    def updatePPGAIN(self, value):
	pitch_p_gain = float(value)/100000
	index = "PP"
	end_index = "e"

	str_pitch_p_gain = index + str(pitch_p_gain) + end_index

	update.write(str_pitch_p_gain)

    def updatePIGAIN(self, value):
	pitch_i_gain = float(value)/100000
	index = "PI"
	end_index = "e"

	str_pitch_i_gain = index + str(pitch_i_gain) + end_index

	update.write(str_pitch_i_gain)
	
    def updatePDGAIN(self, value):
	pitch_d_gain = float(value)/100000
	index = "PD"
	end_index = "e"

	str_pitch_d_gain = index + str(pitch_d_gain) + end_index

	update.write(str_pitch_d_gain)

	

    def updateRPGAIN(self, value):
	roll_p_gain = float(value)/100000
	index = "RP"
	end_index = "e"

	str_roll_p_gain = index + str(roll_p_gain) + end_index

	update.write(str_roll_p_gain)

    def updateRIGAIN(self, value):
	roll_i_gain = float(value)/100000
	index = "RI"
	end_index = "e"

	str_roll_i_gain = index + str(roll_i_gain) + end_index

	update.write(str_roll_i_gain)
	
    def updateRDGAIN(self, value):
	roll_d_gain = float(value)/100000
	index = "RD"
	end_index = "e"

	str_roll_d_gain = index + str(roll_d_gain) + end_index

	update.write(str_roll_d_gain)



    def updateYPGAIN(self, value):
	yaw_p_gain = float(value)/100000
	index = "YP"
	end_index = "e"

	str_yaw_p_gain = index + str(yaw_p_gain) + end_index

	update.write(str_yaw_p_gain)

    def updateYIGAIN(self, value):
	yaw_i_gain = float(value)/100000
	index = "YI"
	end_index = "e"

	str_yaw_i_gain = index + str(yaw_i_gain) + end_index

	update.write(str_yaw_i_gain)
	
    def updateYDGAIN(self, value):
	yaw_d_gain = float(value)/100000
	index = "YD"
	end_index = "e"

	str_yaw_d_gain = index + str(yaw_d_gain) + end_index

	update.write(str_yaw_d_gain)

root = Tk()
root.wm_title('Wireless PID Tuning')
app = App(root)
root.geometry("500x500+0+0")
root.mainloop()
