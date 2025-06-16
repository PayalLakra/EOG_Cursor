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
# DEVIATION_SIGMA = 10       # Number of standard deviations for threshold
# MIN_MOVEMENT_SAMPLES = 10  # Minimum consecutive samples to consider a movement
# COOLDOWN_SAMPLES = 20      # Samples to wait after movement before detecting new one
# SAMPLE_RATE = 250          # Samples per second

# # Data Structures
# circular_buffer = deque(maxlen=BUFFER_SIZE)
# baseline = None
# baseline_std = None
# current_state = "NEUTRAL"
# movement_samples = 0       # Counter for consecutive movement samples
# cooldown_counter = 0       # Counter for cooldown period
# last_movement = None       # Last detected movement type

# # Pattern detection
# state_history = deque(maxlen=4)  # Stores last 4 states for pattern detection

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

# def check_for_complete_movement():
#     """Check if we have a complete movement pattern and return the type"""
#     if len(state_history) == 4:
#         # Check for UP movement pattern: DOWN -> NEUTRAL -> UP -> NEUTRAL
#         if (state_history[0] == "DOWN" and 
#             state_history[1] == "NEUTRAL" and 
#             state_history[2] == "UP" and 
#             state_history[3] == "NEUTRAL"):
#             return "UP"
        
#         # Check for DOWN movement pattern: UP -> NEUTRAL -> DOWN -> NEUTRAL
#         if (state_history[0] == "UP" and 
#             state_history[1] == "NEUTRAL" and 
#             state_history[2] == "DOWN" and 
#             state_history[3] == "NEUTRAL"):
#             return "DOWN"
#     return None

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
#         state_history.append(detected_state)  # Track state for pattern detection

#         # If we're in cooldown, just decrement counter
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
#                     print(detected_state)  # Original output
#                     current_state = detected_state
#                     last_movement = detected_state
#                     cooldown_counter = COOLDOWN_SAMPLES
#                     movement_samples = 0
#         else:
#             # Reset movement counters when neutral
#             if current_state != "NEUTRAL":
#                 print("NEUTRAL")  # Original output
#                 current_state = "NEUTRAL"
#             movement_samples = 0
#             last_movement = None

#         # Check for complete movement patterns after each state is processed
#         movement_type = check_for_complete_movement()
#         if movement_type == "UP":
#             print("1 UP MOVEMENT DETECTED SUCCESSFULLY")  # Additional confirmation
#         elif movement_type == "DOWN":
#             print("1 DOWN MOVEMENT DETECTED SUCCESSFULLY")  # Additional confirmation

# except KeyboardInterrupt:
#     print("Stopped.")



import pylsl
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter, iirnotch

# LSL Setup
print("Searching for EEG stream...")
streams = pylsl.resolve_stream('type', 'EXG')
inlet = pylsl.StreamInlet(streams[0])

# Parameters
BUFFER_SIZE = 500          
BASELINE_SAMPLES = 200     
DEVIATION_THRESHOLD = 10.0  
MIN_MOVEMENT_SAMPLES = 10  
COOLDOWN_SAMPLES = 20      
SAMPLE_RATE = 250          

# Filter parameters
NOTCH_FREQ = 50.0         
NOTCH_Q = 30.0            
LOW_CUT = 1.0             
HIGH_CUT = 20.0           

# Initialize all tracking variables
data_buffer = deque(maxlen=BUFFER_SIZE)
baseline_mean = None
baseline_std = None
current_state = "NEUTRAL"
state_history = deque(maxlen=4)
movement_samples = 0
cooldown_counter = 0
last_movement = None  # This was missing

def initialize_filters():
    # Notch filter
    b_notch, a_notch = iirnotch(NOTCH_FREQ, NOTCH_Q, SAMPLE_RATE)
    
    # Bandpass filter
    nyq = 0.5 * SAMPLE_RATE
    low = LOW_CUT / nyq
    high = HIGH_CUT / nyq
    b_band, a_band = butter(4, [low, high], btype='band')
    
    return (b_notch, a_notch), (b_band, a_band)

def apply_filters(sample, filters, filter_states):
    (b_notch, a_notch), (b_band, a_band) = filters
    notch_state, band_state = filter_states
    
    filtered, new_notch_state = lfilter(b_notch, a_notch, [sample], zi=notch_state)
    filtered, new_band_state = lfilter(b_band, a_band, filtered, zi=band_state)
    
    return filtered[0], (new_notch_state, new_band_state)

# Initialize filters
filters = initialize_filters()
filter_states = (np.zeros(len(filters[0][1])-1), np.zeros(len(filters[1][1])-1))

try:
    print("Starting acquisition...")
    while True:
        # Get new sample
        sample, _ = inlet.pull_sample()
        sample = sample[0]  # Using first channel
        
        # Apply filters
        filtered_sample, filter_states = apply_filters(sample, filters, filter_states)
        data_buffer.append(filtered_sample)
        
        # Establish baseline
        if len(data_buffer) == BASELINE_SAMPLES and baseline_mean is None:
            baseline_mean = np.mean(data_buffer)
            baseline_std = np.std(data_buffer)
            print(f"Baseline established: mean={baseline_mean:.2f}μV, std={baseline_std:.2f}")
            continue
        
        if baseline_mean is None:
            continue
            
        # Detect movement
        deviation = filtered_sample - baseline_mean
        abs_deviation = abs(deviation)
        
        if abs_deviation > DEVIATION_THRESHOLD:
            movement_direction = "UP" if deviation > 0 else "DOWN"
            
            if current_state != movement_direction:
                if last_movement is None or movement_direction == last_movement:
                    movement_samples += 1
                else:
                    movement_samples = 1
                
                if movement_samples >= MIN_MOVEMENT_SAMPLES:
                    print(f"{movement_direction} ", end='', flush=True)
                    state_history.append(movement_direction)
                    current_state = movement_direction
                    last_movement = movement_direction
                    cooldown_counter = COOLDOWN_SAMPLES
                    movement_samples = 0
        else:
            if current_state != "NEUTRAL":
                print("NEUTRAL ", end='', flush=True)
                state_history.append("NEUTRAL")
                current_state = "NEUTRAL"
            movement_samples = 0
            last_movement = None
            
        # Check for patterns
        if len(state_history) == 4:
            pattern = list(state_history)
            if pattern == ["UP", "NEUTRAL", "DOWN", "NEUTRAL"]:
                print("\n>> COMPLETE UP PATTERN DETECTED! <<")
            elif pattern == ["DOWN", "NEUTRAL", "UP", "NEUTRAL"]:
                print("\n>> COMPLETE DOWN PATTERN DETECTED! <<")

except KeyboardInterrupt:
    print("\nStopping acquisition...")