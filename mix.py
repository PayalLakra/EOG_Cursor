# import pylsl
# from collections import deque
# import numpy as np
# import time

# # LSL Setup
# print("Searching for LSL stream...")
# streams = pylsl.resolve_stream('type', "EXG")
# inlet = pylsl.StreamInlet(streams[0])

# # Parameters
# BUFFER_SIZE = 200          # Store last 200 samples
# BASELINE_SAMPLES = 100     # First 100 samples = neutral baseline
# DEVIATION_SIGMA = 10     # Number of standard deviations for threshold
# MIN_MOVEMENT_SAMPLES = 10  # Minimum samples to consider a movement
# NEUTRAL_PRINT_INTERVAL = 1 # Seconds between neutral state prints
# SAMPLE_RATE = 250          # Samples per second

# # Data Structures
# circular_buffer = deque(maxlen=BUFFER_SIZE)
# baseline = None
# baseline_std = None
# current_state = "NEUTRAL"
# last_print_time = time.time()

# def update_baseline_stats():
#     """Calculate baseline mean and standard deviation"""
#     global baseline, baseline_std
#     baseline = np.mean(circular_buffer)
#     baseline_std = np.std(circular_buffer)
#     print(f"Baseline set: {baseline:.2f}μV (±{baseline_std:.2f})")

# def get_movement_type(current_value):
#     deviation = current_value - baseline
#     threshold = DEVIATION_SIGMA * baseline_std
    
#     if deviation > threshold:
#         return "DOWN"
#     elif deviation < -threshold:
#         return "UP"
#     else:
#         return "NEUTRAL"

# try:
#     while True:
#         sample, _ = inlet.pull_sample()
#         sample = sample[0]
#         circular_buffer.append(sample)

#         if len(circular_buffer) == BASELINE_SAMPLES and baseline is None:
#             update_baseline_stats()
#             continue

#         if baseline is None:
#             continue  # Wait until baseline is set

#         # Detect current movement state
#         detected_state = get_movement_type(sample)
#         current_time = time.time()

#         # Only print when state changes
#         if detected_state != current_state:
#             if detected_state == "NEUTRAL":
#                 print("NEUTRAL")
#             elif detected_state == "UP" and current_state != "UP":
#                 print("UP")
#             elif detected_state == "DOWN" and current_state != "DOWN":
#                 print("DOWN")
            
#             current_state = detected_state
#             last_print_time = current_time
        
#         # # Print neutral periodically if we're in neutral state
#         # elif current_state == "NEUTRAL" and (current_time - last_print_time) >= NEUTRAL_PRINT_INTERVAL:
#         #     print("NEUTRAL")
#         #     last_print_time = current_time

# except KeyboardInterrupt:
#     print("Stopped.")





import pylsl
from collections import deque
import numpy as np
import time

# LSL Setup
print("Searching for LSL stream...")
streams = pylsl.resolve_stream('type', "EXG")
inlet = pylsl.StreamInlet(streams[0])

# Parameters
BUFFER_SIZE = 200          # Store last 200 samples(Change is needed)
BASELINE_SAMPLES = 100     # First 100 samples = neutral baseline
DEVIATION_SIGMA = 5        # Number of standard deviations for threshold (Change is needed)
MIN_MOVEMENT_SAMPLES = 10  # Minimum consecutive samples to consider a movement
COOLDOWN_SAMPLES = 20      # Samples to wait after movement before detecting new one
SAMPLE_RATE = 250          # Samples per second

# Data Structures
circular_buffer = deque(maxlen=BUFFER_SIZE)
baseline = None
baseline_std = None
current_state = "NEUTRAL"
movement_samples = 0       # Counter for consecutive movement samples
cooldown_counter = 0       # Counter for cooldown period
last_movement = None       # Last detected movement type

def update_baseline_stats():
    global baseline, baseline_std
    baseline = np.mean(circular_buffer)
    baseline_std = np.std(circular_buffer)
    print(f"Baseline set: {baseline:.2f}μV (±{baseline_std:.2f})")

def get_movement_type(current_value):
    deviation = current_value - baseline
    threshold = DEVIATION_SIGMA * baseline_std
    
    if deviation > threshold:
        return "DOWN"
    elif deviation < -threshold:
        return "UP"
    else:
        return "NEUTRAL"

try:
    while True:
        sample, _ = inlet.pull_sample()
        sample = sample[0]
        circular_buffer.append(sample)

        if len(circular_buffer) == BASELINE_SAMPLES and baseline is None:
            update_baseline_stats()
            continue

        if baseline is None:
            continue  # Wait until baseline is set

        # Detect current movement state
        detected_state = get_movement_type(sample)

        if cooldown_counter > 0:
            cooldown_counter -= 1
            continue

        # Movement validation logic
        if detected_state != "NEUTRAL":
            if detected_state == last_movement or last_movement is None:
                movement_samples += 1
            else:
                # Reset if movement type changes during detection
                movement_samples = 1
            
            # Only confirm movement after minimum consecutive samples
            if movement_samples >= MIN_MOVEMENT_SAMPLES:
                if detected_state != current_state:
                    print(detected_state)
                    current_state = detected_state
                    last_movement = detected_state
                    cooldown_counter = COOLDOWN_SAMPLES
                    movement_samples = 0
        else:
            # Reset movement counters when neutral
            if current_state != "NEUTRAL":
                print("NEUTRAL")
                current_state = "NEUTRAL"
            movement_samples = 0
            last_movement = None

except KeyboardInterrupt:
    print("Stopped.")