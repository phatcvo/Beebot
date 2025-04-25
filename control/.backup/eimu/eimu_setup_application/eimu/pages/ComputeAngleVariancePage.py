import tkinter as tk
import ttkbootstrap as tb
from ttkbootstrap.constants import *

import numpy as np
from termcolor import colored

from eimu.globalParams import g


class ComputeAngleVarFrame(tb.Frame):
  def __init__(self, parentFrame):
    super().__init__(master=parentFrame)

    # intialize parameter
    self.start_process = False
    self.loop_count = 0
    self.no_of_samples = 1000
    self.roll_arr = []
    self.pitch_arr = []
    self.yaw_arr = []

    self.label = tb.Label(self, text="COMPUTE ORIENTATION VARIANCE", font=('Monospace',16, 'bold') ,bootstyle="dark")
  
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

    self.roll_arr = []
    self.pitch_arr = []
    self.yaw_arr = []

    percent = 0.0
    self.textVal.configure(text=f'{percent} %')
    self.progressBar['value'] = percent

  def read_cal_data(self):
    if self.start_process:
      self.no_of_samples = 1000

      roll, pitch, yaw = g.serClient.get('/rpy')

      self.roll_arr.append(roll)
      self.pitch_arr.append(pitch)
      self.yaw_arr.append(yaw)

      self.loop_count += 1
      percent = (self.loop_count*100)/self.no_of_samples
      self.textVal.configure(text=f'{percent} %')
      self.progressBar['value'] = percent

      if percent >= 100.0:
        percent = 100.0
        self.textVal.configure(text=f'{percent} %')
        self.progressBar['value'] = percent
        self.print_computed_variance()
      else:
        self.canvas.after(1, self.read_cal_data)

    else:
      self.reset_all_params()
      self.canvas.after(10, self.compute_variance)

  def print_computed_variance(self):
    roll_variance = np.var(self.roll_arr)
    pitch_variance = np.var(self.pitch_arr)
    yaw_variance = np.var(self.yaw_arr)

    rpy_variance = [ roll_variance, pitch_variance, yaw_variance]
    print(colored("\n---------------------------------------------------------------", 'magenta'))
    print(colored("computed rpy variances:", 'cyan'))
    print(rpy_variance)

    g.serClient.send('/rpy-var', roll_variance, pitch_variance, yaw_variance)
    roll_variance, pitch_variance, yaw_variance = g.serClient.get('/rpy-var')

    rpy_variance = [ roll_variance, pitch_variance, yaw_variance]
    print(colored("stored rpy variances", 'green'))
    print(rpy_variance)
    print(colored("---------------------------------------------------------------", 'magenta'))

  def compute_variance(self):
    if self.start_process:
      self.reset_all_params()
      self.read_cal_data()
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