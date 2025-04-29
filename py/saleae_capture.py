from saleae import automation
import os
from datetime import datetime


class SaleaeCaptureSession:
    def __init__(self, port=10430):
        self.manager = automation.Manager.connect(port=port)
        self.capture = None
        self.output_dir = None

    def start_capture(self, analog_channels=[3, 4, 5, 6, 7], sample_rate=3_125_000):
        # Configure device
        device_config = automation.LogicDeviceConfiguration(
            enabled_analog_channels=analog_channels,
            analog_sample_rate=sample_rate
        )

        # Switch to Manual Capture Mode (no duration limit)
        capture_config = automation.CaptureConfiguration(
            capture_mode=automation.ManualCaptureMode()
        )

        self.capture = self.manager.start_capture(
            device_configuration=device_config,
            capture_configuration=capture_config
        )



    def end_capture(self):
        if self.capture:
            # Stop the manual capture first!
            self.capture.stop()

            # Now wait for the capture to finish saving
            self.capture.wait()

            # Create output directory
            self.output_dir = os.path.join(
                os.getcwd(), 'data_output', f'output-{datetime.now():%Y-%m-%d_%H-%M-%S}'
            )
            os.makedirs(self.output_dir)

            # Save .sal capture
            self.capture.save_capture(filepath=os.path.join(self.output_dir, 'capture.sal'))

            # Export analog CSV
            self.capture.export_raw_data_csv(
                directory=self.output_dir,
                analog_channels=[3, 4, 5, 6, 7],
                analog_downsample_ratio=1000
            )

        self.capture.close()
        self.capture = None


    def close(self):
        self.manager.close()