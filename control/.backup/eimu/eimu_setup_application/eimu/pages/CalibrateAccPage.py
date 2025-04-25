# REFERENCE: https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration/gyro-calibration-with-jupyter
import tkinter as tk
import ttkbootstrap as tb
from ttkbootstrap.constants import *

import matplotlib.pyplot as plt
from collections import deque
import numpy as np
from termcolor import colored

from eimu.globalParams import g


class CalibrateAccFrame(tb.Frame):
  def __init__(self, parentFrame):
    super().__init__(master=parentFrame)

    # intialize parameter
    self.start_process = False
    self.loop_count = 0
    self.no_of_samples = 1000
    self.acc_x = deque(maxlen=self.no_of_samples)
    self.acc_y = deque(maxlen=self.no_of_samples)
    self.acc_z = deque(maxlen=self.no_of_samples)

    self.label = tb.Label(self, text="CALIBRATE ACCELEROMETER", font=('Monospace',16, 'bold') ,bootstyle="dark")
  
    #create widgets to be added to the Fame
    percent = 0.0
    self.textVal = tb.Label(self, text=f'{percent} %', font=('Monospace',20, 'bold'), bootstyle="primary")
    self.progressBar = tb.Progressbar(self, bootstyle="danger striped", mode='determinate',
                                      maximum=100, length=200, value=0.0)

    buttonStyle = tb.Style()
    buttonStyleName = 'primary.TButton'
    buttonStyle.configure(buttonStyleName, font=('Monospace',15,'bold'))
    self.pressButton = tb.Button(self, text="START", style=buttonStyleName,
                                 command=self.change_btn_state)
    
    self.canvasFrame = tb.Frame(self)
    
    #add created widgets to Frame
    self.label.pack(side='top', pady=(20,50))
    self.textVal.pack(side='top', expand=True, fill='y')
    self.progressBar.pack(side='top', expand=True, fill='x', padx=50)
    self.pressButton.pack(side='top', fill='y')
    self.canvasFrame.pack(side='top', expand=True, fill='both', pady=(10,0))

    #create widgets to be added to the canvasFame
    self.canvas = tb.Canvas(self.canvasFrame, width=300, height=10, autostyle=False ,bg="#FFFFFF", relief='solid')

    #add created widgets to canvasFame
    self.canvas.pack(side='left', expand=True, fill='both')

    # start process
    self.compute_variance()

  def reset_all_params(self):
    self.loop_count = 0
    self.no_of_samples = 1000
    self.acc_x = deque(maxlen=self.no_of_samples)
    self.acc_y = deque(maxlen=self.no_of_samples)
    self.acc_z = deque(maxlen=self.no_of_samples)

    percent = 0.0
    self.textVal.configure(text=f'{percent} %')
    self.progressBar['value'] = percent

  def read_data(self):
    if self.start_process:
      ax, ay, az = g.serClient.get("/acc-raw")

      self.acc_x.append(ax)
      self.acc_y.append(ay)
      self.acc_z.append(az)

      self.loop_count += 1
      percent = (self.loop_count*100)/self.no_of_samples
      self.textVal.configure(text=f'{percent} %')
      self.progressBar['value'] = percent

      if percent >= 100.0:
        percent = 100.0
        self.textVal.configure(text=f'{percent} %')
        self.progressBar['value'] = percent
        self.plot_calibrated_data()
      else:
        self.canvas.after(1, self.read_data)

    else:
      self.reset_all_params()
      self.canvas.after(10, self.compute_variance)

  def plot_calibrated_data(self):
    ax_offset = self.average(self.acc_x)
    ay_offset = self.average(self.acc_y)
    az_offset = (self.average(self.acc_z) - 9.8)

    acc_calibration = [ ax_offset, ay_offset, az_offset ]
    print(colored("\n---------------------------------------------------------------", 'magenta'))
    print(colored("computed acc offsets in m/s^2::", 'cyan'))
    print(acc_calibration)

    g.serClient.send('/acc-off', ax_offset, ay_offset, az_offset)
    ax_offset, ay_offset, az_offset = g.serClient.get('/acc-off')

    acc_calibration = [ ax_offset, ay_offset, az_offset ]
    print(colored("stored acc offsets in m/s^2:", 'green'))
    print(acc_calibration)
    print(colored("---------------------------------------------------------------", 'magenta'))


    fig, (uncal, cal) = plt.subplots(nrows=2)

    # Clear all axis
    uncal.cla()
    cal.cla()
    t = np.linspace(0, len(self.acc_x), len(self.acc_x))


    # plot uncalibrated data
    uncal.plot(t, self.acc_x, color='r')
    uncal.plot(t, self.acc_y, color='g')
    uncal.plot(t, self.acc_z, color='b')
    uncal.title.set_text("Uncalibrated Acc")
    uncal.set(ylabel='g')

    uncal.grid(which = "major", linewidth = 0.5)
    uncal.grid(which = "minor", linewidth = 0.2)
    uncal.minorticks_on()

    # plot calibrated data
    cal.plot(t, [x - acc_calibration[0] for x in self.acc_x], color='r')
    cal.plot(t, [y - acc_calibration[1] for y in self.acc_y], color='g')
    cal.plot(t, [z - acc_calibration[2] for z in self.acc_z], color='b')
    cal.title.set_text("Calibrated Acc")
    cal.set(ylabel='g')

    cal.grid(which = "major", linewidth = 0.5)
    cal.grid(which = "minor", linewidth = 0.2)
    cal.minorticks_on()

    fig.tight_layout()
    plt.show()

  def compute_variance(self):
    if self.start_process:
      self.reset_all_params()
      self.read_data()
    else:
      self.reset_all_params()
      self.canvas.after(10, self.compute_variance)

  def change_btn_state(self):
    if self.start_process:
      self.start_process = False
      self.pressButton.configure(text='START')

    else:
      self.start_process = True
      self.pressButton.configure(text='STOP')

  def average(self, val):
      ans = 0
      for i in val:
        ans= ans + i
      
      ans = ans/len(val)
      
      return ans