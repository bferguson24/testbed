from file_formatting import *

    
# file_path = get_file_path(MotorSpeeds.SPEED_0, MotorOrientation.HORIZONTAL, SoilCompaction.COMPACT)
# read_csv(file_path)
# file_config = FileConfig(
#     speed = MotorSpeed.SPEED_100,
#     orientation= MotorOrientation.VERTICAL,
#     soil = SoilCompaction.COMPACT
# )
# data = Data(file_config)


#TO DO : Calculate magnitude of F and a and plot those instead. Adjust x axis for Bar graph plot. Make bar graph separate data output. Clean up plots. 

full_file_list = {}

for speed in MotorSpeed:
    file_config = FileConfig(
        speed=speed,
        orientation=MotorOrientation.HORIZONTAL,
        soil=SoilCompaction.LOOSE
    )
    try:
        data = Data(file_config)
        data.FFT("fy")  # All plots go on the same figure

        full_file_list[speed] = data  # Store the Data object with MotorSpeed as the key
    except FileNotFoundError:
        print(f"Data file not found for config: {file_config}")



speeds = []
max_magnitudes = []
labels = []
colors = []
frequency_labels = []  # Add this here
f_0 = 65.05 

color_mapping = {
    0: '#1F77B4',   # Blue
    20: '#FF7F0E',  # Orange
    30: '#2CA02C',  # Green
    40: '#D62728',  # Red
    50: '#9467BD',  # Purple
    60: '#8C564B',  # Brown
    70: '#E377C2',  # Pink
    100: '#7F7F7F'  # Gray
}

for speed, data in full_file_list.items():
    peak_info = data.fft_peaks.get("fy", {})
    freq = peak_info.get("frequency")
    mag = peak_info.get("magnitude")

    if freq is not None and mag is not None:
        speed_int = int(speed.value)
        speeds.append(speed.value)
        max_magnitudes.append(mag)

        # Add speed and frequency (stacked label)
        freq_val = dutycycle_2_freq(speed_int)
        labels.append(f"{speed.value}%\n[{freq_val:.1f} Hz]")  # Two-line label

        colors.append(color_mapping.get(speed_int, 'gray'))
        frequency_labels.append(f"{freq_val:.1f} Hz")  # Optional annotation above bar



plt.figure(figsize=(10, 5))
bars = plt.bar(labels, max_magnitudes, color=colors, width=0.4)
plt.title("Peak FFT Amplitudes of Fx vs. Vibration Speed")
plt.xlabel("Vibration Speed (%)\n [Frequency]", labelpad=20)
plt.ylabel("Peak Amplitude")
plt.ylim(0, max(max_magnitudes) * 1.2)  # Scale Y-axis to 120% of max height
plt.grid(axis='y', linestyle='--', alpha=0.7)


plt.tight_layout()
plt.savefig("fft_max_peaks.png")
plt.show()




# # Plot and label
# for speed, data in full_file_list.items():
#     label = f"{speed.value} % Vibration" 
#     data.plot_outputs(
#         "fx",
#         plot_type="low_pass_filter",
#         speed=str(speed),
#         title="Fx comparison - FFT",
#         save_as_png=False,  # Avoid saving individual plots
#         label=label
#     )

# plt.legend()
# plt.title("Fx - Time Scale Plot")
# plt.xlabel("Time [s]")
# plt.ylabel("Force [N]")
# plt.grid(True)
# plt.tight_layout()
# plt.savefig("fx_time_comparison.png")  # Save the composite plot
# plt.show()



speeds = []
max_magnitudes = []
labels = []





























