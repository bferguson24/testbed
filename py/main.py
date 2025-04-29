import serial
import time
from SerialPacket import *
from main_gui import ScoopGUI
from scoop_control import Scoop
from saleae_capture import SaleaeCaptureSession
import tkinter as tk



root = tk.Tk()
scoop = Scoop()
session = SaleaeCaptureSession() 



gui = ScoopGUI(root, scoop)
root.mainloop()

