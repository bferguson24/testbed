import pandas as pd
import numpy as np
import sys
import os
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt


#Update later*
csv_path = Path("test_data") / "multichannel_test.csv"


def read_csv_and_report_memory(filepath, sort_columns = None, ascending = True):
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        return

    # Read the CSV file into a DataFrame
    df = pd.read_csv(filepath)

    print("Columns in CSV:", df.columns.tolist())   

    # if sort_columns:
    #     if isinstance(sort_columns, str):


    

    # # Print first few rows for preview
    # print("First few rows of the file:")
    # print(df.head())

    # # Get memory usage
    # mem_usage = df.memory_usage(deep=True).sum()  # in bytes
    # mem_usage_mb = mem_usage / (1024 ** 2)

    # print(f"\nMemory used by DataFrame: {mem_usage_mb:.2f} MB ({mem_usage:,} bytes)")
    return df

def butter_low_pass_filter(df, value_col, time_col, cutoff_freq=1.0, order=2):

    if value_col not in df.columns or time_col not in df.columns:
        raise ValueError("Required columns not found in DataFrame.")

    # Estimate sampling rate from time column
    time = df[time_col].values
    signal = df[value_col].values
    dt = np.mean(np.diff(time))  # average time step
    fs = 1.0 / dt  # sampling frequency in Hz

    # Normalize cutoff frequency (Nyquist frequency = fs / 2)
    nyq = 0.5 * fs
    normal_cutoff = cutoff_freq / nyq

    # Create Butterworth filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)

    # Apply zero-phase filter
    filtered_signal = filtfilt(b, a, signal)

    return pd.Series(filtered_signal, index=df.index)

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



data = read_csv_and_report_memory(csv_path)
data['filtered_x'] = butter_low_pass_filter(data, value_col = 'fx', time_col = 'time_s', cutoff_freq= 10.0)

plot_data(data, time_col='time_s', value_col='fx', new_figure=True, label='Raw fx')
plot_data(data, time_col='time_s', value_col='filtered_x', new_figure=False, label='Filtered fx')
plt.show()




