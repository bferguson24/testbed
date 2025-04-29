import tkinter as tk
from scoop_control import Scoop

class ScoopGUI:
    def __init__(self, root, scoop):
        self.root = root
        self.scoop = scoop

        self.angle_var = tk.StringVar()
        self.vibration_var = tk.StringVar()

        self.toggle_data_button = None 
        self.create_widgets()
        self.bind_variables()


    def create_widgets(self):
        # tk.Button(self.root, text="Idle", command=self.scoop.idle).grid(row=0, column=0, padx=10, pady=10)
        # tk.Button(self.root, text="Start", command=self.scoop.start).grid(row=0, column=1, padx=10, pady=10)
        # tk.Button(self.root, text="Stop", command=self.scoop.stop).grid(row=0, column=2, padx=10, pady=10)
        tk.Button(self.root, text="Home", command=self.scoop.home).grid(row=0, column=0, padx=10, pady=10)
        tk.Button(self.root, text="Run Dig", command=self.scoop.dig_sequence).grid(row=0, column=1, padx=10, pady=10)
        tk.Button(self.root, text="Manual Control", command=self.scoop.manual_control).grid(row=0, column=2, padx=10, pady=10)

        self.toggle_data_button = tk.Button(
            self.root,
            text="Data Read ON",
            command=self.toggle_data_read,
            bg="lightgreen"
        )
        self.toggle_data_button.grid(row=1, column=1, padx=10, pady=10)

        # tk.Label(self.root, text="Angle:").grid(row=2, column=0, padx=10, pady=10)
        # tk.Entry(self.root, textvariable=self.angle_var).grid(row=2, column=1, padx=10, pady=10)

        # tk.Label(self.root, text="Max Vibration:").grid(row=3, column=0, padx=10, pady=10)
        # tk.Entry(self.root, textvariable=self.vibration_var).grid(row=3, column=1, padx=10, pady=10)


    def bind_variables(self):
        self.angle_var.trace_add("write", self.update_scoop_values)
        self.vibration_var.trace_add("write", self.update_scoop_values)

    def toggle_data_read(self):
        self.scoop.record_data = not self.scoop.record_data
        self.toggle_data_button.config(
            text=f"Data Read {'ON' if self.scoop.record_data else 'OFF'}",
            bg="lightgreen" if self.scoop.record_data else "lightcoral"
        )

    def update_scoop_values(self, *_):
        try:
            angle = float(self.angle_var.get())
        except ValueError:
            angle = 0.0

        try:
            vibration = float(self.vibration_var.get())
        except ValueError:
            vibration = 0.0

        self.scoop.update_values(angle, vibration)
        print(f"Updated Scoop values: angle={angle}, max vibration={vibration}")


def run_gui():
    root = tk.Tk()
    root.title("Scoop Testbed Control")

    angle_var = tk.StringVar()
    vibration_var = tk.StringVar()

    scoop = Scoop()  # Create an instance of your Scoop control object

    # Initialize the GUI class
    gui = ScoopGUI(root, scoop, angle_var, vibration_var)

    # Run the Tkinter event loop
    root.mainloop()

if __name__ == "__main__":
    run_gui()