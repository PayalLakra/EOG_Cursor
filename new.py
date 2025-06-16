# import pylsl
# from collections import deque
# import numpy as np
# import time

# # LSL Setup
# print("Looking for LSL stream...")
# streams = pylsl.resolve_stream('type', "EXG")
# inlet = pylsl.StreamInlet(streams[0])

# # Parameters
# BUFFER_SIZE = 200          # Store last 200 samples
# BASELINE_SAMPLES = 100     # First 100 samples = neutral baseline
# DEVIATION_SIGMA = 2.0      # Standard deviations for threshold
# DEBOUNCE_TIME = 0.3        # Seconds to wait between detections
# ADAPTATION_RATE = 0.01     # How quickly baseline adapts (0-1)

# # Data Structures
# circular_buffer = deque(maxlen=BUFFER_SIZE)
# baseline = None
# baseline_std = None
# last_detection_time = 0
# current_state = "NEUTRAL"

# def calculate_thresholds():
#     """Calculate dynamic thresholds based on baseline statistics"""
#     positive_thresh = baseline + DEVIATION_SIGMA * baseline_std
#     negative_thresh = baseline - DEVIATION_SIGMA * baseline_std
#     return positive_thresh, negative_thresh

# try:
#     while True:
#         sample, _ = inlet.pull_sample()
#         sample = sample[0]
#         circular_buffer.append(sample)

#         # Initial Baseline Calculation
#         if len(circular_buffer) == BASELINE_SAMPLES and baseline is None:
#             baseline = np.mean(circular_buffer)
#             baseline_std = np.std(circular_buffer)
#             print(f"Baseline: {baseline:.2f} ± {baseline_std:.2f}")
#             current_state = "NEUTRAL"
#             continue

#         # Adaptive Baseline (continually adjust to slow drifts)
#         if baseline is not None and current_state == "NEUTRAL":
#             baseline = baseline * (1-ADAPTATION_RATE) + sample * ADAPTATION_RATE
#             baseline_std = baseline_std * 0.99 + np.abs(sample - baseline) * 0.01

#         # Movement Detection
#         if baseline is not None and baseline_std is not None:
#             now = time.time()
#             pos_thresh, neg_thresh = calculate_thresholds()
            
#             if now - last_detection_time > DEBOUNCE_TIME:
#                 if sample > pos_thresh:
#                     if current_state != "DOWN":
#                         print(f"DOWN (+{sample-baseline:.2f}μV)")
#                         current_state = "DOWN"
#                         last_detection_time = now
#                 elif sample < neg_thresh:
#                     if current_state != "UP":
#                         print(f"UP (-{baseline-sample:.2f}μV)")
#                         current_state = "UP"
#                         last_detection_time = now
#                 elif current_state != "NEUTRAL":
#                     print("NEUTRAL")
#                     current_state = "NEUTRAL"

# except KeyboardInterrupt:
#     print("Stopped.")



import pylsl
from collections import deque
import numpy as np
import time

# LSL Setup
print("Looking for LSL stream...")
streams = pylsl.resolve_stream('type', "EXG")
inlet = pylsl.StreamInlet(streams[0])

# Parameters
BUFFER_SIZE = 200          # Store last 200 samples
BASELINE_SAMPLES = 100     # First 100 samples = neutral baseline
DEVIATION_SIGMA = 3.0      # Number of standard deviations for threshold
MIN_MOVEMENT_SAMPLES = 10  # Minimum samples to consider a movement
SAMPLE_RATE = 250          # Samples per second (adjust based on your device)

# Data Structures
circular_buffer = deque(maxlen=BUFFER_SIZE)
baseline = None
baseline_std = None
last_state = "NEUTRAL"
movement_samples = 0

def update_baseline_stats():
    """Calculate baseline mean and standard deviation"""
    global baseline, baseline_std
    baseline = np.mean(circular_buffer)
    baseline_std = np.std(circular_buffer)
    print(f"Baseline set: {baseline:.2f}μV (±{baseline_std:.2f})")

def get_movement_type(current_value):
    """Determine movement direction based on current value"""
    deviation = current_value - baseline
    threshold = DEVIATION_SIGMA * baseline_std
    
    if deviation > threshold:
        return "Up"
    elif deviation < -threshold:
        return "Down"
    else:
        return "NEUTRAL"

try:
    while True:
        sample, _ = inlet.pull_sample()
        sample = sample[0]  # Assuming single channel
        circular_buffer.append(sample)

        # Initial baseline setup
        if len(circular_buffer) == BASELINE_SAMPLES and baseline is None:
            update_baseline_stats()
            continue

        if baseline is None:
            continue  # Wait until baseline is set

        # Detect current movement state
        current_movement = get_movement_type(sample)

        # State transition logic
        if current_movement != last_state:
            if current_movement != "NEUTRAL":
                # New movement detected
                movement_samples = 1
            else:
                # Returning to neutral
                if movement_samples >= MIN_MOVEMENT_SAMPLES:
                    print(f"{last_state} movement completed")
                movement_samples = 0
        elif current_movement != "NEUTRAL":
            movement_samples += 1

        # Confirm movement after minimum duration
        if (movement_samples == MIN_MOVEMENT_SAMPLES and 
            current_movement != "NEUTRAL"):
            pass
            # print(f"{current_movement} movement detected")

        last_state = current_movement

        if current_movement == "NEUTRAL" and int(time.time()) % 5 == 0:
            print("NEUTRAL state")

except KeyboardInterrupt:
    print("Stopped.")

# Neutral state is detected perfectly but Up and Down are not detected perfectly.