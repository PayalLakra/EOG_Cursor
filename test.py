import pylsl
from collections import deque
import numpy as np

# LSL Setup
print("Looking for LSL stream...")
streams = pylsl.resolve_stream('type',"EXG")
inlet = pylsl.StreamInlet(streams[0])

# Parameters
BUFFER_SIZE = 200          # Store last 200 samples
BASELINE_SAMPLES = 100     # First 100 samples = neutral baseline
DEVIATION_TOLERANCE = 0.5  # How far from baseline
DEBOUNCE_TIME = 0.3        # Seconds to wait between detections

# Data Structures
circular_buffer = deque(maxlen=BUFFER_SIZE)  # Fixed-size buffer for signal
baseline = None                              # Neutral level

# Main Loop
try:
    while True:
        sample, _ = inlet.pull_sample()
        sample = sample[0]  # Assuming single-channel data
        circular_buffer.append(sample)

        # Step 1: Compute Baseline (First BASELINE_SAMPLES)
        if len(circular_buffer) == BASELINE_SAMPLES and baseline is None:
            baseline = np.mean(circular_buffer)
            print(f"Baseline (neutral) set to: {baseline}")
            continue  # Skip detection until baseline is set

        # Step 2: Detect Movements (Only After Baseline is Set)
        if baseline is not None:
            current_value = circular_buffer[-1]  # Latest sample

            # Check for significant deviations from baseline
            is_positive_dev = (current_value - baseline) > DEVIATION_TOLERANCE
            is_negative_dev = (baseline - current_value) > DEVIATION_TOLERANCE

            # Classify Movement
            if is_negative_dev:
                print("UP (Negative → Positive)")
                circular_buffer.clear()  # Reset buffer after detection
            elif is_positive_dev:
                print("DOWN (Positive → Negative)")
                circular_buffer.clear()
            else:
                print("NEUTRAL")

except KeyboardInterrupt:
    print("Stopped.")

# Works fine for Up and Down but prints many Up and Down messages. and no neutral message.