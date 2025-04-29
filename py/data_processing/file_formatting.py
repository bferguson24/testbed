import pandas as pd
import numpy as np
import itertools
import sys
import os
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from enum import Enum

class MotorSpeed(str, Enum):
    SPEED_0 = "0"
    SPEED_20 = "20"
    SPEED_30 = "30"
    SPEED_40 = "40"
    SPEED_50 = "50"
    SPEED_60 = "60"
    SPEED_70 = "70"
    SPEED_100 = "100"

class MotorOrientation(str, Enum):
    HORIZONTAL = "horiz"
    VERTICAL = "vert"

class SoilCompaction(str, Enum):
    COMPACT = "compact"
    LOOSE = "loose"

class FileConfig:

    def __init__(self, speed: MotorSpeed, orientation: MotorOrientation, soil:SoilCompaction):
        self.speed = speed
        self.orientation = orientation
        self.soil = soil
    
    def __str__(self):
        return self.speed.value + "_" + self.orientation.value + "_" + self.soil.value

class Data:
    def __init__(self, file: FileConfig, data_folder: str = "data_output"):
        df = get_data(file, data_folder)

        if df is None:
            raise FileNotFoundError(f"Data file not found for config: {file}")

        self.fx_vref = df["Channel 4"].iloc[:50].mean()
        self.fy_vref = df["Channel 3"].iloc[:50].mean()
        self.ax_vref = df["Channel 6"].iloc[:50].mean()
        self.ay_vref = df["Channel 5"].iloc[:50].mean()
        self.az_vref = df["Channel 7"].iloc[:50].mean()

        self.time = df["Time [s]"]        
        self.fx = volt_2_newton(df["Channel 4"], self.fx_vref)
        self.fy = volt_2_newton(df["Channel 3"], self.fy_vref)
        self.f_sum = np.sqrt(self.fx**2 + self.fy**2)

        self.ax = volt_2_g(df["Channel 6"], self.ax_vref)
        self.ay = volt_2_g(df["Channel 5"], self.ay_vref)
        self.az = volt_2_g(df["Channel 7"], self.az_vref)
        self.a_sum = np.sqrt(self.ax**2 + self.ay**2 + self.az**2)

        self.filtered_Fx = None
        self.filtered_Fy = None

        self.fft_peaks = {}  

        self.fft_Fx = None
        self.fft_Fy = None
        self.fft_ax = None
        self.fft_ay = None
        self.fft_az = None

    def FFT(self, signal_attr: str, cutoff_freq: float = 5.0):
        signal = getattr(self, signal_attr, None)

        if signal is None:
            raise AttributeError(f"{signal_attr} is not a valid attribute of the Data object.")
        
        dt = np.mean(np.diff(self.time))
        sampling_rate = 1.0 / dt

        n = len(signal)
        freqs = np.fft.rfftfreq(n, d=1/sampling_rate)
        fft_values = np.fft.rfft(signal)
        magnitude = np.abs(fft_values)

        valid_indices = freqs >= cutoff_freq
        freqs = freqs[valid_indices]
        magnitude = magnitude[valid_indices]

        if len(magnitude) > 0:
            max_idx = np.argmax(magnitude)
            max_freq = freqs[max_idx]
            max_magnitude = magnitude[max_idx]
            print(f"[{signal_attr}] Max FFT magnitude: {max_magnitude:.3f} at {max_freq:.2f} Hz")

            # Store max info in fft_peaks
            self.fft_peaks[signal_attr] = {
                "frequency": max_freq,
                "magnitude": max_magnitude
            }
        else:
            print(f"[{signal_attr}] No FFT data above {cutoff_freq} Hz.")
            self.fft_peaks[signal_attr] = {
                "frequency": None,
                "magnitude": None
            }

        fft_result_attr = f"fft_{signal_attr}"
        setattr(self, fft_result_attr, (freqs, magnitude))

        return freqs, magnitude




    def low_pass_filter(
        self,
        signal_attr: str,
        cutoff_freq=1.0,
        order=2,
        plot=False,
        store=True,
        new_figure=True,
        plot_unfiltered_data=False
    ):
        signal = getattr(self, signal_attr, None)

        if signal is None:
            raise AttributeError(f"{signal_attr} is not a valid attribute of the Data object.")
        
        time = self.time
        dt = np.mean(np.diff(time))  
        fs = 1.0 / dt 

        nyq = 0.5 * fs
        normal_cutoff = cutoff_freq / nyq

        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        filtered_signal = filtfilt(b, a, signal)

        if store:
            filtered_signal_attr = f"filtered_{signal_attr}"
            setattr(self, filtered_signal_attr, filtered_signal)

    
        if plot:
            if new_figure:
                plt.figure()

            # Use two distinct colors
            color_unfiltered = 'tab:blue'
            color_filtered = 'tab:orange'

            if plot_unfiltered_data:
                plt.plot(time, signal, label=f'Original {signal_attr}', color=color_unfiltered)
            plt.plot(time, filtered_signal, label=f'Filtered {signal_attr}', color=color_filtered)

            plt.title(f"Low-pass Filter ({signal_attr}, Cutoff: {cutoff_freq} Hz)")
            plt.xlabel("Time (s)")
            plt.ylabel("Signal Amplitude")
            plt.legend()
            plt.grid(True)

            if new_figure:
                plt.show()

        return filtered_signal
    
    def plot_outputs(self, signal_attr: str, plot_type: str = "FFT", speed: str = None, title: str = None, save_as_png: bool = False, label: str = None):
        if plot_type == "FFT":
            freqs, magnitude = getattr(self, f"fft_{signal_attr}", (None, None))
            if freqs is None or magnitude is None:
                print(f"FFT data not available for {signal_attr}")
                return

            plot_label = label if label else f"FFT of {signal_attr}"
            if speed and not label:
                plot_label += f" (Speed: {speed})"

            plt.plot(freqs, magnitude, label=plot_label)

        elif plot_type == "low_pass_filter":
            filtered_signal = getattr(self, f"filtered_{signal_attr}", None)
            if filtered_signal is None:
                print(f"Filtered data not available for {signal_attr}")
                return

            time = self.time
            plot_label = label if label else f"Filtered {signal_attr}"
            if speed and not label:
                plot_label += f" (Speed: {speed})"

            plt.plot(time, filtered_signal, label=plot_label)

        else:
            print(f"Unsupported plot type: {plot_type}")
            return

        if title:
            plt.title(title)

        plt.xlabel("Frequency (Hz)" if plot_type == "FFT" else "Time (s)")
        plt.ylabel("Magnitude" if plot_type == "FFT" else signal_attr)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        if save_as_png:
            screenshots_folder = "data_processing/screenshots"
            os.makedirs(screenshots_folder, exist_ok=True)

            filename = f"{signal_attr}_Speed_{speed}_{plot_type}.png".replace(" ", "_")
            filepath = os.path.join(screenshots_folder, filename)
            plt.savefig(filepath)
            print(f"Saved plot: {filepath}")


def volt_2_g(v_in, v_ref):
    scale = 1000/40 # [40mV/g]
    accel_g = (v_in - v_ref) * scale
    return accel_g

def volt_2_newton(v_in, v_ref):
    k = 110.719
    newton = (v_in - v_ref) * k
    return newton

def dutycycle_2_freq(duty_in):
    scale = 2.0827 
    offset = 28.221
    freq = (duty_in * scale) - offset 
    return freq

def read_csv(filepath, sort_columns = None, ascending = True):
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        return

    # Read the CSV file into a DataFrame
    df = pd.read_csv(filepath)
    print(f"Read data from {os.path.basename(filepath)}")
    return df

def get_data(file_config: FileConfig, data_folder: str):
    excel_file_name = str(file_config) + ".csv"  

    complete_file_path = os.path.join(data_folder, excel_file_name)

    if not os.path.exists(complete_file_path):
        print(f"File not found: {complete_file_path}")
        return None

    return read_csv(complete_file_path)

def plot_data(data, time_col, value_col, new_figure=True, label=None):

    if time_col not in data.columns or value_col not in data.columns:
        raise ValueError(f"Columns {time_col} or {value_col} not found in DataFrame.")
    
    if new_figure:
            plt.figure(figsize=(10, 6))

    plot_label = label if label else value_col
    plt.plot(data[time_col], data[value_col], label=plot_label, linewidth=2)
    plt.xlabel(time_col)
    plt.ylabel(value_col)
    plt.title(f'Data over {time_col}')
    plt.grid(True)
    plt.legend()
