import threading
import time
import numpy as np
from collections import deque
from scipy.signal import butter, lfilter, lfilter_zi, iirnotch
import pylsl
import pyautogui

LSL_TYPE = "EXG"
SAMPLE_RATE = 500  # Hz
BUFFER_SIZE = 500  # For movement detection
BASELINE_SAMPLES = 100

# Left/Right detection (channel 1)
LR_DEVIATION_SIGMA = 10
LR_MIN_MOVEMENT_SAMPLES = 8
LR_COOLDOWN_SAMPLES = 200
LR_MOVE_PIXELS = 80

# Up/Down detection (channel 0)
UD_DEVIATION_SIGMA = 8
UD_MIN_MOVEMENT_SAMPLES = 8
UD_COOLDOWN_SAMPLES = 200
UD_MOVE_PIXELS = 80

# Movement validation
SEQUENCE_WINDOW = 20  # Samples to look back for sequence validation
MIN_SEQUENCE_CONFIDENCE = 0.7  # Minimum ratio of dominant movement in sequence

# Filtering
NOTCH_FREQ = 50.0
NOTCH_Q = 20.0
BANDPASS_LOW = 0.5
BANDPASS_HIGH = 10.0

# Double blink detection (channel 2)
DB_WINDOW_SEC = 3  # seconds of data to keep in buffer
DB_FILTER_HZ = 5.0  # lowpass filter cutoff for blink
DB_MIN_INTERBLINK_GAP = 0.08  # minimum time between blinks (s)
DB_MAX_INTERBLINK_GAP = 0.4   # maximum time between blinks (s)
DB_COOLDOWN = 1.0  # seconds to wait after a double blink

def get_lsl_inlet():
    print("Searching for LSL stream...")
    streams = pylsl.resolve_stream('type', LSL_TYPE)
    if not streams:
        raise RuntimeError("No LSL streams found!")
    inlet = pylsl.StreamInlet(streams[0])
    print(f"Connected to LSL stream: {streams[0].name()}")
    return inlet

class MovementDetector:
    def __init__(self, name, channel, deviation_sigma, min_movement_samples, cooldown_samples, move_pixels):
        self.name = name
        self.channel = channel
        self.deviation_sigma = deviation_sigma
        self.min_movement_samples = min_movement_samples
        self.cooldown_samples = cooldown_samples
        self.move_pixels = move_pixels
        
        # Detection state
        self.circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.baseline = None
        self.baseline_std = None
        self.current_state = "NEUTRAL"
        self.movement_samples = 0
        self.cooldown_counter = 0
        self.last_movement = None
        
        # Sequence validation
        self.movement_sequence = deque(maxlen=SEQUENCE_WINDOW)
        self.last_valid_movement = None
        self.last_movement_time = 0
        
    def update_baseline(self):
        if len(self.circular_buffer) == BASELINE_SAMPLES and self.baseline is None:
            self.baseline = np.median(self.circular_buffer)
            self.baseline_std = np.std(self.circular_buffer)
            print(f"[{self.name}] Baseline set: {self.baseline:.2f}μV (±{self.baseline_std:.2f})")
            return True
        return False
    
    def detect_movement(self, value):
        if self.baseline is None:
            return "NEUTRAL"
            
        deviation = value - self.baseline
        threshold = self.deviation_sigma * self.baseline_std
        
        if deviation < -threshold:
            return "NEGATIVE"  # LEFT for LR, DOWN for UD
        elif deviation > threshold:
            return "POSITIVE"  # RIGHT for LR, UP for UD
        else:
            return "NEUTRAL"
    
    def validate_sequence(self, detected_state):
        """Validate if the detected movement is part of a valid sequence"""
        if len(self.movement_sequence) < 5:  # Need minimum samples for validation
            return True
            
        # Count occurrences of each state in recent sequence
        recent_sequence = list(self.movement_sequence)[-10:]  # Last 10 samples
        state_counts = {}
        for state in recent_sequence:
            state_counts[state] = state_counts.get(state, 0) + 1
        
        # Calculate confidence for the detected state
        total_samples = len(recent_sequence)
        detected_count = state_counts.get(detected_state, 0)
        confidence = detected_count / total_samples
        
        # If we have a clear dominant movement, validate it
        if confidence >= MIN_SEQUENCE_CONFIDENCE:
            return True
            
        # Check for the expected sequence pattern
        if len(recent_sequence) >= 4:
            # Look for pattern: NEUTRAL -> MOVEMENT -> NEUTRAL
            if (recent_sequence[-4] == "NEUTRAL" and 
                recent_sequence[-3] == detected_state and
                recent_sequence[-2] == detected_state and
                recent_sequence[-1] == detected_state):
                return True
                
        return False
    
    def process_sample(self, value):
        self.circular_buffer.append(value)
        
        # Update baseline if needed
        if self.update_baseline():
            return None
            
        # Detect movement
        detected_state = self.detect_movement(value)
        self.movement_sequence.append(detected_state)
        
        # Handle cooldown
        if self.cooldown_counter > 0:
            self.cooldown_counter -= 1
            return None
            
        # Process movement detection
        if detected_state != "NEUTRAL":
            if detected_state == self.last_movement or self.last_movement is None:
                self.movement_samples += 1
            else:
                self.movement_samples = 1
                
            if self.movement_samples >= self.min_movement_samples:
                # Validate the sequence before confirming movement
                if self.validate_sequence(detected_state):
                    if detected_state != self.current_state:
                        self.current_state = detected_state
                        self.last_movement = detected_state
                        self.cooldown_counter = self.cooldown_samples
                        self.movement_samples = 0
                        self.last_movement_time = time.time()
                        
                        # Return the movement for processing
                        return detected_state
        else:
            if self.current_state != "NEUTRAL":
                self.movement_sequence.append("NEUTRAL")
                self.current_state = "NEUTRAL"
            self.movement_samples = 0
            self.last_movement = None
            
        return None

def left_right_worker(inlet):
    detector = MovementDetector(
        name="LR", 
        channel=1, 
        deviation_sigma=LR_DEVIATION_SIGMA,
        min_movement_samples=LR_MIN_MOVEMENT_SAMPLES,
        cooldown_samples=LR_COOLDOWN_SAMPLES,
        move_pixels=LR_MOVE_PIXELS
    )
    
    while True:
        sample, _ = inlet.pull_sample(timeout=0.1)
        if sample is None or len(sample) < 2:
            continue
            
        value = sample[1]  # channel 1 for left/right
        movement = detector.process_sample(value)
        
        if movement == "NEGATIVE":  # LEFT
            pyautogui.moveRel(-LR_MOVE_PIXELS, 0)
            print("LEFT SUCCESS: Cursor moved left.")
        elif movement == "POSITIVE":  # RIGHT
            pyautogui.moveRel(LR_MOVE_PIXELS, 0)
            print("RIGHT SUCCESS: Cursor moved right.")

def up_down_worker(inlet):
    detector = MovementDetector(
        name="UD", 
        channel=0, 
        deviation_sigma=UD_DEVIATION_SIGMA,
        min_movement_samples=UD_MIN_MOVEMENT_SAMPLES,
        cooldown_samples=UD_COOLDOWN_SAMPLES,
        move_pixels=UD_MOVE_PIXELS
    )
    
    while True:
        sample, _ = inlet.pull_sample(timeout=0.1)
        if sample is None or len(sample) < 1:
            continue
            
        value = sample[0]  # channel 0 for up/down
        movement = detector.process_sample(value)
        
        if movement == "POSITIVE":  # UP
            pyautogui.moveRel(0, -UD_MOVE_PIXELS)
            print("UP SUCCESS: Cursor moved up.")
        elif movement == "NEGATIVE":  # DOWN
            pyautogui.moveRel(0, UD_MOVE_PIXELS)
            print("DOWN SUCCESS: Cursor moved down.")

def double_blink_worker(inlet):
    buffer_size = int(SAMPLE_RATE * DB_WINDOW_SEC)
    eog_data = np.zeros(buffer_size)
    current_index = 0
    b, a = butter(4, DB_FILTER_HZ / (0.5 * SAMPLE_RATE), btype='low')
    zi = lfilter_zi(b, a)
    detected_peaks = deque(maxlen=buffer_size)
    start_time = time.time()
    last_double_blink_time = 0
    while True:
        samples, _ = inlet.pull_chunk(timeout=0.1, max_samples=30)
        if samples:
            for sample in samples:
                if len(sample) < 1:
                    continue
                eog_data[current_index] = sample[2]  # channel 2 for blink
                current_index = (current_index + 1) % buffer_size
            filtered_eog, zi = lfilter(b, a, eog_data, zi=zi)
            if time.time() - start_time >= 2:
                mean_signal = np.mean(filtered_eog)
                stdev_signal = np.std(filtered_eog)
                threshold = mean_signal + (1.5 * stdev_signal)
                window_size = 1 * SAMPLE_RATE
                start_index = current_index - window_size
                if start_index < 0:
                    start_index = 0
                end_index = current_index
                filtered_window = filtered_eog[start_index:end_index]
                # Peak detection
                peaks = []
                prev_peak_time = None
                min_peak_gap = 0.1
                for i in range(1, len(filtered_window) - 1):
                    if (filtered_window[i] > filtered_window[i - 1] and
                        filtered_window[i] > filtered_window[i + 1] and
                        filtered_window[i] > threshold):
                        current_peak_time = i / SAMPLE_RATE
                        if prev_peak_time is not None:
                            time_gap = current_peak_time - prev_peak_time
                            if time_gap < min_peak_gap:
                                continue
                        peaks.append(i)
                        prev_peak_time = current_peak_time
                for peak in peaks:
                    full_peak_index = start_index + peak
                    peak_time = time.time() - (current_index - full_peak_index) / SAMPLE_RATE
                    detected_peaks.append((full_peak_index, peak_time))
                # Double blink detection
                if len(detected_peaks) >= 2:
                    last_peak_index, last_peak_time = detected_peaks[-1]
                    prev_peak_index, prev_peak_time = detected_peaks[-2]
                    time_diff = last_peak_time - prev_peak_time
                    if (DB_MIN_INTERBLINK_GAP <= time_diff <= DB_MAX_INTERBLINK_GAP and
                        time.time() - last_double_blink_time > DB_COOLDOWN):
                        # pyautogui.click(button='right', clicks=2)
                        pyautogui.click()  # Left button, 1 click (tap)
                        print("DOUBLE BLINK SUCCESS: Double right click.")
                        last_double_blink_time = time.time()
        else:
            time.sleep(0.01)

if __name__ == "__main__":
    inlet = get_lsl_inlet()
    t_lr = threading.Thread(target=left_right_worker, args=(inlet,), daemon=True)
    t_ud = threading.Thread(target=up_down_worker, args=(inlet,), daemon=True)
    t_db = threading.Thread(target=double_blink_worker, args=(inlet,), daemon=True)
    t_ud.start()
    t_lr.start()
    t_db.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")