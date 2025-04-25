# REFERENCE: https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration/gyro-calibration-with-jupyter
import tkinter as tk
import ttkbootstrap as tb
from ttkbootstrap.constants import *

import matplotlib.pyplot as plt
from collections import deque
import numpy as np
from termcolor import colored

from eimu.globalParams import g


class CalibrateGyroFrame(tb.Frame):
  def __init__(self, parentFrame):
    super().__init__(master=parentFrame)

    # intialize parameter
    self.start_process = False
    self.loop_count = 0
    self.no_of_samples = 1000
    self.gyro_x = deque(maxlen=self.no_of_samples)
    self.gyro_y = deque(maxlen=self.no_of_samples)
    self.gyro_z = deque(maxlen=self.no_of_samples)

    self.label = tb.Label(self, text="CALIBRATE GYROSCOPE", font=('Monospace',16, 'bold') ,bootstyle="dark")
  
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
    self.gyro_x = deque(maxlen=self.no_of_samples)
    self.gyro_y = deque(maxlen=self.no_of_samples)
    self.gyro_z = deque(maxlen=self.no_of_samples)

    percent = 0.0
    self.textVal.configure(text=f'{percent} %')
    self.progressBar['value'] = percent

  def read_data(self):
    if self.start_process:
      gx, gy, gz = g.serClient.get("/gyro-raw")

      self.gyro_x.append(gx)
      self.gyro_y.append(gy)
      self.gyro_z.append(gz)

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
    min_x = min(self.gyro_x)
    max_x = max(self.gyro_x)
    min_y = min(self.gyro_y)
    max_y = max(self.gyro_y)
    min_z = min(self.gyro_z)
    max_z = max(self.gyro_z)

    gx_offset = (max_x + min_x) / 2
    gy_offset = (max_y + min_y) / 2
    gz_offset = (max_z + min_z) / 2

    gyro_calibration = [ gx_offset, gy_offset, gz_offset]
    print(colored("\n---------------------------------------------------------------", 'magenta'))
    print(colored("computed gyro offsets in rad/s:", 'cyan'))
    print(gyro_calibration)

    g.serClient.send('/gyro-off', gx_offset, gy_offset, gz_offset)
    gx_offset, gy_offset, gz_offset = g.serClient.get('/gyro-off')

    gyro_calibration = [ gx_offset, gy_offset, gz_offset]
    print(colored("stored gyro offsets in rad/s:", 'green'))
    print(gyro_calibration)
    print(colored("---------------------------------------------------------------", 'magenta'))


    fig, (uncal, cal) = plt.subplots(2, 1)

    # Clear all axis
    uncal.cla()
    cal.cla()
    t = np.linspace(0, len(self.gyro_x), len(self.gyro_x))


    # plot uncalibrated data
    uncal.set_ylim([-1,1])
    uncal.grid(which = "major", linewidth = 0.5)
    uncal.grid(which = "minor", linewidth = 0.2)
    uncal.minorticks_on()

    uncal.plot(t, self.gyro_x, color='r')
    uncal.plot(t, self.gyro_y, color='g')
    uncal.plot(t, self.gyro_z, color='b')
    uncal.title.set_text("Uncalibrated Gyro")
    uncal.set(ylabel='rad/s')

    # plot calibrated data
    cal.set_ylim([-1,1])
    cal.grid(which = "major", linewidth = 0.5)
    cal.grid(which = "minor", linewidth = 0.2)
    cal.minorticks_on()

    cal.plot(t, [x - gyro_calibration[0] for x in self.gyro_x], color='r')
    cal.plot(t, [y - gyro_calibration[1] for y in self.gyro_y], color='g')
    cal.plot(t, [z - gyro_calibration[2] for z in self.gyro_z], color='b')
    cal.title.set_text("Calibrated Gyro")
    cal.set(ylabel='rad/s')

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