# import pylsl
# import pyautogui
# from collections import deque

# class EOGMouseController:
#     def __init__(self):
#         pyautogui.FAILSAFE = False
        
#         print("Looking for EOG stream...")
#         streams = pylsl.resolve_stream('name', 'NPG')
#         self.inlet = pylsl.StreamInlet(streams[0])
#         print("Stream found and connected.")
        
#         self.ch1_prev = None
#         self.ch2_prev = None
#         self.ch1_sequence = []
#         self.ch2_sequence = []
#         self.blink_buffer = deque(maxlen=5)
        
#         self.UP_PATTERN = ['up', 'down']
#         self.DOWN_PATTERN = ['down', 'up']
#         self.LEFT_PATTERN = ['up', 'down']
#         self.RIGHT_PATTERN = ['down', 'up']
        
#     def detect_movement(self, current, previous, channel):
#         if previous is None:
#             return None
            
#         direction = None
#         if current > previous:
#             direction = 'up'
#         elif current < previous:
#             direction = 'down'
        
#         if channel == 1:
#             self.ch1_sequence.append(direction)
#             if len(self.ch1_sequence) > 2:
#                 self.ch1_sequence.pop(0)
                
#             # Check for patterns
#             if self.ch1_sequence == self.UP_PATTERN:
#                 print("Moving cursor UP")
#                 pyautogui.move(0, -20)  # Move up
#                 self.ch1_sequence = []
#                 return 'up'
#             elif self.ch1_sequence == self.DOWN_PATTERN:
#                 print("Moving cursor DOWN")
#                 pyautogui.move(0, 20)  # Move down
#                 self.ch1_sequence = []
#                 return 'down'
                
#         elif channel == 2:
#             self.ch2_sequence.append(direction)
#             if len(self.ch2_sequence) > 2:
#                 self.ch2_sequence.pop(0)
                
#             if self.ch2_sequence == self.LEFT_PATTERN:
#                 print("Moving cursor LEFT")
#                 pyautogui.move(-50, 0)  # Move left
#                 self.ch2_sequence = []
#                 return 'left'
#             elif self.ch2_sequence == self.RIGHT_PATTERN:
#                 print("Moving cursor RIGHT")
#                 pyautogui.move(50, 0)  # Move right
#                 self.ch2_sequence = []
#                 return 'right'
        
#         return None
    
#     def detect_blink(self, blink_value):
#         self.blink_buffer.append(blink_value)
        
#         # Simple blink detection - look for two consecutive significant changes
#         if len(self.blink_buffer) < 2:
#             return False
            
#         # Check for two consecutive large changes (either direction)
#         changes = [abs(self.blink_buffer[i] - self.blink_buffer[i-1]) 
#                   for i in range(1, len(self.blink_buffer))]
        
#         if len(changes) >= 2 and all(c > 50 for c in changes[-2:]):
#             print("Double blink detected - clicking mouse")
#             pyautogui.click()
#             self.blink_buffer.clear()
#             return True
            
#         return False
    
#     def run(self):
#         try:
#             print("Starting EOG Mouse Controller. Press Ctrl+C to stop.")
#             while True:
#                 # Get a new sample from the LSL stream
#                 sample, timestamp = self.inlet.pull_sample()
#                 blink_ch = sample[0]  # Channel 1 for blinks
#                 vert_ch = sample[1]     # Channel 2 for vertical
#                 horiz_ch = sample[2]    # Channel 3 for horizontal
                
#                 # Detect blinks
#                 self.detect_blink(blink_ch)
                
#                 # Detect vertical movements (Channel 2)
#                 self.detect_movement(vert_ch, self.ch1_prev, 1)
#                 self.ch1_prev = vert_ch
                
#                 # Detect horizontal movements (Channel 3)
#                 self.detect_movement(horiz_ch, self.ch2_prev, 2)
#                 self.ch2_prev = horiz_ch
                
#         except KeyboardInterrupt:
#             print("Stopping EOG Mouse Controller")
#         except Exception as e:
#             print(f"Error: {e}")

# if __name__ == "__main__":
#     controller = EOGMouseController()
#     controller.run()


import pylsl
import pyautogui
import time
from collections import deque

class EOGMouseController:
    def __init__(self):
        self.MOVEMENT_DELAY = 0.8  # seconds between movements
        self.BLINK_THRESHOLD = 50  # threshold for blink detection
        self.VERTICAL_THRESHOLD = 0.5  # minimum change for vertical movement
        self.HORIZONTAL_THRESHOLD = 0.5  # minimum change for horizontal movement
        self.VERTICAL_SPEED = 25    # pixels to move vertically
        self.HORIZONTAL_SPEED = 40  # pixels to move horizontally
        
        # Initialize system
        pyautogui.FAILSAFE = False
        self.last_action_time = time.time()
        
        print("Looking for EOG stream...")
        streams = pylsl.resolve_stream('name', 'NPG')
        self.inlet = pylsl.StreamInlet(streams[0])
        print("Stream found and connected.")
        
        self.prev_ch1 = None  # For blink/vertical (Channel 1)
        self.prev_ch2 = None  # For horizontal (Channel 2)
        self.blink_buffer = deque(maxlen=3)  # Stores recent blink channel values

    def classify_movement(self, ch1, ch2):
        if self.prev_ch1 is not None:
            blink_diff = abs(ch1 - self.prev_ch1)
            self.blink_buffer.append(blink_diff)
            
            if (len(self.blink_buffer) >= 2 and 
                all(d > self.BLINK_THRESHOLD for d in list(self.blink_buffer)[-2:])):
                return 'blink'
        
        if self.prev_ch1 is not None:
            vert_change = ch1 - self.prev_ch1
            if abs(vert_change) > self.VERTICAL_THRESHOLD:
                return 'up' if vert_change > 0 else 'down'
        
        if self.prev_ch2 is not None:
            horiz_change = ch2 - self.prev_ch2
            if abs(horiz_change) > self.HORIZONTAL_THRESHOLD:
                return 'right' if horiz_change > 0 else 'left'
        
        return None

    def execute_action(self, action):
        if action == 'up':
            print("Moving UP")
            pyautogui.move(0, -self.VERTICAL_SPEED)
        elif action == 'down':
            print("Moving DOWN")
            pyautogui.move(0, self.VERTICAL_SPEED)
        elif action == 'left':
            print("Moving LEFT")
            pyautogui.move(-self.HORIZONTAL_SPEED, 0)
        elif action == 'right':
            print("Moving RIGHT")
            pyautogui.move(self.HORIZONTAL_SPEED, 0)
        elif action == 'blink':
            print("BLINK - Clicking mouse")
            pyautogui.click()
        
        self.last_action_time = time.time()

    def run(self):
        try:
            print("Starting EOG Mouse Controller. Press Ctrl+C to stop.")
            print(f"Config: BlinkTh={self.BLINK_THRESHOLD}, VertTh={self.VERTICAL_THRESHOLD}, "
                  f"HorizTh={self.HORIZONTAL_THRESHOLD}, Delay={self.MOVEMENT_DELAY}s")
            
            while True:
                if time.time() - self.last_action_time < self.MOVEMENT_DELAY:
                    time.sleep(0.05)
                    continue
                sample, timestamp = self.inlet.pull_sample()
                ch1, ch2 = sample[0], sample[1]
                
                action = self.classify_movement(ch1, ch2)
                
                if action:
                    self.execute_action(action)
                
                self.prev_ch1 = ch1
                self.prev_ch2 = ch2

                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nStopping EOG Mouse Controller")
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    controller = EOGMouseController()
    controller.run()


# import numpy as np
# import pylsl
# import time
# import pyautogui
# from threading import Thread, Lock
# from collections import deque
# from scipy.signal import butter, lfilter

# class ChannelProcessor:
#     def __init__(self, channel_type, buffer_size=50):
#         self.channel_type = channel_type  # 'vertical' or 'horizontal'
#         self.buffer = deque(maxlen=buffer_size)
#         self.baseline = 0
#         self.threshold = 30  # Initial threshold, will auto-adjust
#         self.b, self.a = butter(4, [0.5/250, 30/250], btype='band')  # Filter for 500Hz
        
#         # Movement detection parameters
#         self.blink_threshold = 50 if channel_type == 'vertical' else 0
#         self.move_cooldown = 0.3
#         self.last_detection = 0
        
#         # For cursor control
#         self.cursor_speed = 25
#         self.lock = Lock()

#     def process_sample(self, sample):
#         # Filter the sample
#         filtered = lfilter(self.b, self.a, [sample])[0]
#         self.buffer.append(filtered)
        
#         # Update baseline (median of recent samples)
#         if len(self.buffer) > 10:
#             self.baseline = np.median(self.buffer)
            
#         # Detect movements
#         current_time = time.time()
#         if len(self.buffer) == self.buffer.maxlen and current_time - self.last_detection > self.move_cooldown:
#             signal = np.array(self.buffer) - self.baseline
#             max_val = np.max(np.abs(signal))
            
#             # Vertical channel processing
#             if self.channel_type == 'vertical':
#                 if max_val > self.blink_threshold and len(self.buffer) < 15:
#                     self._execute_action('blink')
#                     self.last_detection = current_time
#                 elif max_val > self.threshold:
#                     direction = 'up' if np.mean(signal[-5:]) > 0 else 'down'
#                     self._execute_action(direction)
#                     self.last_detection = current_time
            
#             # Horizontal channel processing
#             elif self.channel_type == 'horizontal' and max_val > self.threshold:
#                 direction = 'left' if np.mean(signal[-5:]) > 0 else 'right'
#                 self._execute_action(direction)
#                 self.last_detection = current_time

#     def _execute_action(self, action):
#         with self.lock:
#             print(f"{self.channel_type} channel detected: {action}")
#             if action == 'blink':
#                 pyautogui.click()
#             elif action == 'up':
#                 pyautogui.move(0, -self.cursor_speed)
#             elif action == 'down':
#                 pyautogui.move(0, self.cursor_speed)
#             elif action == 'left':
#                 pyautogui.move(-self.cursor_speed, 0)
#             elif action == 'right':
#                 pyautogui.move(self.cursor_speed, 0)

# def channel_thread(processor, inlet, channel_idx):
#     while True:
#         sample, _ = inlet.pull_sample()
#         processor.process_sample(sample[channel_idx])
#         time.sleep(0.002)  # ~500Hz sampling

# def main():
#     print("Connecting to LSL stream...")
#     try:
#         streams = pylsl.resolve_stream('name', 'NPG')
#         inlet = pylsl.StreamInlet(streams[0])
#         print(f"Connected to {streams[0].name()}")
#     except Exception as e:
#         print(f"LSL connection failed: {e}")
#         return

#     # Create processors for each channel
#     vertical_processor = ChannelProcessor('vertical')
#     horizontal_processor = ChannelProcessor('horizontal')

#     # Start processing threads
#     Thread(target=channel_thread, args=(vertical_processor, inlet, 0), daemon=True).start()
#     Thread(target=channel_thread, args=(horizontal_processor, inlet, 1), daemon=True).start()

#     print("EOG cursor control running. Press Ctrl+C to stop.")
#     try:
#         while True: time.sleep(1)
#     except KeyboardInterrupt:
#         print("\nStopping EOG control...")

# if __name__ == "__main__":
#     main()