# WORKS PRINTS UP SUCCESS AND DOWN SUCCESS
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
BUFFER_SIZE = 250          
BASELINE_SAMPLES = 100     
DEVIATION_SIGMA = 10        
MIN_MOVEMENT_SAMPLES = 8   
COOLDOWN_SAMPLES = 15      
SAMPLE_RATE = 250          

# Filter parameters
NOTCH_FREQ = 50.0         
NOTCH_Q = 20.0            
BANDPASS_LOW = 1.0        
BANDPASS_HIGH = 20.0      

# Data Structures
circular_buffer = deque(maxlen=BUFFER_SIZE)
baseline = None
baseline_std = None
current_state = "NEUTRAL"
movement_samples = 0
cooldown_counter = 0
last_movement = None

# Movement sequence tracking - needs to store at least 4 elements
movement_sequence = deque(maxlen=4)  # Large enough buffer

# Filter state variables
notch_b, notch_a = None, None
bandpass_b, bandpass_a = None, None
filter_state_notch = None
filter_state_bandpass = None

def initialize_filters():
    global notch_b, notch_a, bandpass_b, bandpass_a, filter_state_notch, filter_state_bandpass
    notch_b, notch_a = iirnotch(NOTCH_FREQ, NOTCH_Q, SAMPLE_RATE)
    nyq = 0.5 * SAMPLE_RATE
    low = BANDPASS_LOW / nyq
    high = BANDPASS_HIGH / nyq
    bandpass_b, bandpass_a = butter(2, [low, high], btype='band')
    filter_state_notch = np.zeros(max(len(notch_a), len(notch_b)) - 1)
    filter_state_bandpass = np.zeros(max(len(bandpass_a), len(bandpass_b)) - 1)

def apply_filters(sample):
    global filter_state_notch, filter_state_bandpass
    if filter_state_notch[0] == -1:
        filtered, filter_state_notch = lfilter(notch_b, notch_a, [sample], zi=None)
    else:
        filtered, filter_state_notch = lfilter(notch_b, notch_a, [sample], zi=filter_state_notch)
    if filter_state_bandpass[0] == -1:
        filtered, filter_state_bandpass = lfilter(bandpass_b, bandpass_a, filtered, zi=None)
    else:
        filtered, filter_state_bandpass = lfilter(bandpass_b, bandpass_a, filtered, zi=filter_state_bandpass)
    return filtered[0]

def update_baseline_stats():
    global baseline, baseline_std
    baseline = np.median(circular_buffer)
    baseline_std = np.std(circular_buffer)
    print(f"Baseline set: {baseline:.2f}μV (±{baseline_std:.2f})")

def get_movement_type(current_value):
    deviation = current_value - baseline
    threshold = DEVIATION_SIGMA * baseline_std
    if deviation < -threshold:
        return "DOWN"
    elif deviation > threshold:
        return "UP"
    else:
        return "NEUTRAL"

def check_movement_completion():
    # Need at least 4 elements to check the pattern
    if len(movement_sequence) < 4:
        return False
    
    sequence = list(movement_sequence)
    
    # Check for UP completion pattern: UP NEUTRAL DOWN NEUTRAL
    for i in range(len(sequence)-3):
        if (sequence[i] == "UP" and 
            sequence[i+1] == "NEUTRAL" and 
            sequence[i+2] == "DOWN" and 
            sequence[i+3] == "NEUTRAL"):
            print("UP SUCCESS")
            # Remove everything up to and including this pattern
            for _ in range(i+4):
                if movement_sequence:
                    movement_sequence.popleft()
            return True
    
    # Check for DOWN completion pattern: DOWN NEUTRAL UP NEUTRAL
    for i in range(len(sequence)-3):
        if (sequence[i] == "DOWN" and 
            sequence[i+1] == "NEUTRAL" and 
            sequence[i+2] == "UP" and 
            sequence[i+3] == "NEUTRAL"):
            print("DOWN SUCCESS")
            for _ in range(i+4):
                if movement_sequence:
                    movement_sequence.popleft()
            return True
    
    return False

# Initialize filters
initialize_filters()

try:
    while True:
        sample, _ = inlet.pull_sample()
        sample = sample[0]
        filtered_sample = sample  # Using raw signal
        
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
                    # print(f"{detected_state} ")
                    movement_sequence.append(detected_state)
                    current_state = detected_state
                    last_movement = detected_state
                    cooldown_counter = COOLDOWN_SAMPLES
                    movement_samples = 0
                    check_movement_completion()
        else:
            if current_state != "NEUTRAL":
                # print("NEUTRAL")
                movement_sequence.append("NEUTRAL")
                current_state = "NEUTRAL"
                check_movement_completion()
            movement_samples = 0
            last_movement = None

except KeyboardInterrupt:
    print("Stopped.")