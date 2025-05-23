import tkinter as tk
import ttkbootstrap as tb
from ttkbootstrap.constants import *

import beer
from beer.globalParams import g
from beer.pages.SerialConnectPage import SerialConnectFrame
from beer.pages.MainAppPage import MainAppFrame



class App(tb.Window):
  def __init__(self, title, size):
    super().__init__()
    self.title(title)
    self.geometry(f'{size[0]}x{size[1]}')
    self.resizable(False,False)

    self.windowFrame = tb.Frame(self)
    self.windowFrame.pack(side="left", expand=True, fill='both')

    #####################################################################################
    self.connectToPortFrame = SerialConnectFrame(self.windowFrame, next_func=self.startMainApp)
    self.connectToPortFrame.pack(side="left", expand=True, fill="both", pady=(0,10))
    #####################################################################################

  def startMainApp(self):
    self.delete_pages()
    self.mainAppFrame = MainAppFrame(self.windowFrame)
    self.mainAppFrame.pack(side="left", expand=True, fill="both", pady=(0,10))

  def delete_pages(self):
    for frame in self.windowFrame.winfo_children():
      frame.destroy()


def main():
  g.app = App(title="Bee Driver Application", size=(900,650))
  g.app.mainloop()


if __name__ == "__main__":
  main()