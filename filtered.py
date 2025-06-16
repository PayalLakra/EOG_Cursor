# import pylsl
# from collections import deque
# import numpy as np
# import time
# from scipy.signal import butter, lfilter, iirnotch

# # LSL Setup
# print("Searching for LSL stream...")
# streams = pylsl.resolve_stream('type', "EXG")
# inlet = pylsl.StreamInlet(streams[0])

# # Parameters
# BUFFER_SIZE = 200          # Store last 200 samples(Change is needed)
# BASELINE_SAMPLES = 100     # First 100 samples = neutral baseline
# DEVIATION_SIGMA = 5        # Number of standard deviations for threshold (Change is needed)
# MIN_MOVEMENT_SAMPLES = 10  # Minimum consecutive samples to consider a movement
# COOLDOWN_SAMPLES = 20      # Samples to wait after movement before detecting new one
# SAMPLE_RATE = 250          # Samples per second

# # Filter parameters
# NOTCH_FREQ = 50.0         # Frequency to notch out
# NOTCH_Q = 30.0            # Quality factor for notch filter
# BANDPASS_LOW = 0.1        # Low cutoff frequency
# BANDPASS_HIGH = 30.0      # High cutoff frequency

# # Data Structures
# circular_buffer = deque(maxlen=BUFFER_SIZE)
# baseline = None
# baseline_std = None
# current_state = "NEUTRAL"
# movement_samples = 0       # Counter for consecutive movement samples
# cooldown_counter = 0       # Counter for cooldown period
# last_movement = None       # Last detected movement type

# # Filter state variables
# notch_b = None
# notch_a = None
# bandpass_b = None
# bandpass_a = None
# filter_state_notch = None
# filter_state_bandpass = None

# def initialize_filters():
#     global notch_b, notch_a, bandpass_b, bandpass_a, filter_state_notch, filter_state_bandpass
    
#     # Design notch filter (50 Hz)
#     notch_b, notch_a = iirnotch(NOTCH_FREQ, NOTCH_Q, SAMPLE_RATE)
    
#     # Design bandpass filter (0.1-10 Hz)
#     nyq = 0.5 * SAMPLE_RATE
#     low = BANDPASS_LOW / nyq
#     high = BANDPASS_HIGH / nyq
#     bandpass_b, bandpass_a = butter(4, [low, high], btype='band')
    
#     # Initialize filter states
#     filter_state_notch = np.zeros(max(len(notch_a), len(notch_b)) - 1)
#     filter_state_bandpass = np.zeros(max(len(bandpass_a), len(bandpass_b)) - 1)

# def apply_filters(sample):
#     global filter_state_notch, filter_state_bandpass
    
#     # Apply notch filter
#     if filter_state_notch[0] == -1:  # First sample
#         filtered, filter_state_notch = lfilter(notch_b, notch_a, [sample], zi=None)
#     else:
#         filtered, filter_state_notch = lfilter(notch_b, notch_a, [sample], zi=filter_state_notch)
    
#     # Apply bandpass filter
#     if filter_state_bandpass[0] == -1:  # First sample
#         filtered, filter_state_bandpass = lfilter(bandpass_b, bandpass_a, filtered, zi=None)
#     else:
#         filtered, filter_state_bandpass = lfilter(bandpass_b, bandpass_a, filtered, zi=filter_state_bandpass)
    
#     return filtered[0]

# def update_baseline_stats():
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

# # Initialize filters
# initialize_filters()

# try:
#     while True:
#         sample, _ = inlet.pull_sample()
#         sample = sample[0]
        
#         # Apply filters to the sample
#         filtered_sample = apply_filters(sample)
        
#         circular_buffer.append(filtered_sample)

#         if len(circular_buffer) == BASELINE_SAMPLES and baseline is None:
#             update_baseline_stats()
#             continue

#         if baseline is None:
#             continue  # Wait until baseline is set

#         # Detect current movement state
#         detected_state = get_movement_type(filtered_sample)

#         if cooldown_counter > 0:
#             cooldown_counter -= 1
#             continue

#         # Movement validation logic
#         if detected_state != "NEUTRAL":
#             if detected_state == last_movement or last_movement is None:
#                 movement_samples += 1
#             else:
#                 # Reset if movement type changes during detection
#                 movement_samples = 1
            
#             # Only confirm movement after minimum consecutive samples
#             if movement_samples >= MIN_MOVEMENT_SAMPLES:
#                 if detected_state != current_state:
#                     print(detected_state)
#                     current_state = detected_state
#                     last_movement = detected_state
#                     cooldown_counter = COOLDOWN_SAMPLES
#                     movement_samples = 0
#         else:
#             # Reset movement counters when neutral
#             if current_state != "NEUTRAL":
#                 print("NEUTRAL")
#                 current_state = "NEUTRAL"
#             movement_samples = 0
#             last_movement = None

# except KeyboardInterrupt:
#     print("Stopped.")

# Not detecting Down Movements due to filter



# Detects Up as UP,NEUTRAL,DOWN,NEUTRAL
import pylsl
from collections import deque
import numpy as np
import time
from scipy.signal import butter, lfilter, iirnotch

# LSL Setup
print("Searching for LSL stream...")
streams = pylsl.resolve_stream('type', "EXG")
inlet = pylsl.StreamInlet(streams[0])

# Parameters
BUFFER_SIZE = 250          # Increased buffer size
BASELINE_SAMPLES = 100     # First 100 samples = neutral baseline
DEVIATION_SIGMA = 10        # More sensitive threshold
MIN_MOVEMENT_SAMPLES = 8   # Reduced minimum samples
COOLDOWN_SAMPLES = 15      # Reduced cooldown
SAMPLE_RATE = 250          # Samples per second

# Filter parameters - modified to be less aggressive
NOTCH_FREQ = 50.0         # Frequency to notch out
NOTCH_Q = 20.0            # Reduced quality factor
BANDPASS_LOW = 1.0        # Increased low cutoff (was 0.1)
BANDPASS_HIGH = 20.0      # Increased high cutoff (was 10.0)

# Data Structures
circular_buffer = deque(maxlen=BUFFER_SIZE)
baseline = None
baseline_std = None
current_state = "NEUTRAL"
movement_samples = 0
cooldown_counter = 0
last_movement = None

# Filter state variables
notch_b, notch_a = None, None
bandpass_b, bandpass_a = None, None
filter_state_notch = None
filter_state_bandpass = None

def initialize_filters():
    global notch_b, notch_a, bandpass_b, bandpass_a, filter_state_notch, filter_state_bandpass
    
    # Less aggressive notch filter
    notch_b, notch_a = iirnotch(NOTCH_FREQ, NOTCH_Q, SAMPLE_RATE)
    
    # Wider bandpass filter
    nyq = 0.5 * SAMPLE_RATE
    low = BANDPASS_LOW / nyq
    high = BANDPASS_HIGH / nyq
    bandpass_b, bandpass_a = butter(2, [low, high], btype='band')  # Reduced order
    
    # Initialize filter states
    filter_state_notch = np.zeros(max(len(notch_a), len(notch_b)) - 1)
    filter_state_bandpass = np.zeros(max(len(bandpass_a), len(bandpass_b)) - 1)

def apply_filters(sample):
    global filter_state_notch, filter_state_bandpass
    
    # Option 1: Apply only notch filter
    if filter_state_notch[0] == -1:
        filtered, filter_state_notch = lfilter(notch_b, notch_a, [sample], zi=None)
    else:
        filtered, filter_state_notch = lfilter(notch_b, notch_a, [sample], zi=filter_state_notch)
    
    # Option 2: Skip bandpass filter entirely for this application
    # filtered = filtered[0]  # Uncomment if using only notch
    
    # Option 3: Apply both but with less aggressive settings
    if filter_state_bandpass[0] == -1:
        filtered, filter_state_bandpass = lfilter(bandpass_b, bandpass_a, filtered, zi=None)
    else:
        filtered, filter_state_bandpass = lfilter(bandpass_b, bandpass_a, filtered, zi=filter_state_bandpass)
    
    return filtered[0]

def update_baseline_stats():
    global baseline, baseline_std
    baseline = np.median(circular_buffer)  # Using median instead of mean
    baseline_std = np.std(circular_buffer)
    print(f"Baseline set: {baseline:.2f}μV (±{baseline_std:.2f})")

def get_movement_type(current_value):
    deviation = current_value - baseline
    threshold = DEVIATION_SIGMA * baseline_std
    
    # More sensitive to downward movements
    if deviation < -threshold:  # 20% more sensitive to downward movements
        return "DOWN"
    elif deviation > threshold:
        return "UP"
    else:
        return "NEUTRAL"

# Initialize filters
initialize_filters()

try:
    while True:
        sample, _ = inlet.pull_sample()
        sample = sample[0]
        
        # Option 4: Try raw signal or just notch filtering
        # filtered_sample = apply_filters(sample)  # Full filtering
        filtered_sample = sample  # Try without filtering first
        
        circular_buffer.append(filtered_sample)

        if len(circular_buffer) == BASELINE_SAMPLES and baseline is None:
            update_baseline_stats()
            continue

        if baseline is None:
            continue

        # Detect current movement state
        detected_state = get_movement_type(filtered_sample)

        if cooldown_counter > 0:
            cooldown_counter -= 1
            continue

        # Movement validation
        if detected_state != "NEUTRAL":
            if detected_state == last_movement or last_movement is None:
                movement_samples += 1
            else:
                movement_samples = 1
            
            if movement_samples >= MIN_MOVEMENT_SAMPLES:
                if detected_state != current_state:
                    print(f"{detected_state} ")
                    current_state = detected_state
                    last_movement = detected_state
                    cooldown_counter = COOLDOWN_SAMPLES
                    movement_samples = 0
        else:
            if current_state != "NEUTRAL":
                print("NEUTRAL")
                current_state = "NEUTRAL"
            movement_samples = 0
            last_movement = None

except KeyboardInterrupt:
    print("Stopped.")