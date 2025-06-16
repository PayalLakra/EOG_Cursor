import numpy as np
from pylsl import StreamInlet, resolve_stream
from config import *
import time

def calibrate_thresholds(calibration_duration=10):
    print(f"Calibrating for {calibration_duration} seconds...")
    print("Please keep your eyes in neutral position (looking straight ahead)")
    
    # Connect to LSL
    streams = resolve_stream('type', STREAM_TYPE)
    if STREAM_NAME:
        streams = [s for s in streams if s.name() == STREAM_NAME]
    
    if not streams:
        raise RuntimeError("No EOG stream found")
    
    inlet = StreamInlet(streams[0])
    
    h_samples = []
    v_samples = []
    start_time = time.time()
    
    while time.time() - start_time < calibration_duration:
        sample, _ = inlet.pull_sample()
        h_samples.append(abs(sample[0]))
        v_samples.append(abs(sample[1]))
        time.sleep(0.01)
    
    h_thresh = np.mean(h_samples) + 2 * np.std(h_samples)
    v_thresh = np.mean(v_samples) + 2 * np.std(v_samples)
    blink_thresh = np.mean(v_samples) + 4 * np.std(v_samples)
    
    print("\nCalibration results:")
    print(f"Horizontal threshold: {h_thresh:.3f}")
    print(f"Vertical threshold: {v_thresh:.3f}")
    print(f"Blink threshold: {blink_thresh:.3f}")
    
    # Update config
    with open('config.py', 'r') as f:
        lines = f.readlines()
    
    with open('config.py', 'w') as f:
        for line in lines:
            if line.startswith('HORIZONTAL_THRESHOLD ='):
                f.write(f"HORIZONTAL_THRESHOLD = {h_thresh:.3f}\n")
            elif line.startswith('VERTICAL_THRESHOLD ='):
                f.write(f"VERTICAL_THRESHOLD = {v_thresh:.3f}\n")
            elif line.startswith('BLINK_THRESHOLD ='):
                f.write(f"BLINK_THRESHOLD = {blink_thresh:.3f}\n")
            else:
                f.write(line)
    
    print("Configuration updated successfully")

if __name__ == "__main__":
    calibrate_thresholds()