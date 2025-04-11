import serial
import time
from SerialPacket import *
from main_gui import ScoopGUI
from scoop_control import Scoop
from saleae import Saleae
import tkinter as tk


root = tk.Tk()
scoop = Scoop()
gui = ScoopGUI(root, scoop)

root.mainloop()
