# LSL Stream Configuration
STREAM_TYPE = 'EXG'          # Changed from 'EOG' to 'EXG'
STREAM_NAME = 'NPG'          # Specific stream name
STREAM_CHANNELS = 3          # Number of channels
STREAM_SFREQ = 500           # Sampling frequency
STREAM_DTYPE = 'int16'       # Data type
STREAM_SOURCE_ID = 'npg1234' # Source ID

HORIZONTAL_CHANNEL = 1       # channel for left/right eye movements
VERTICAL_CHANNEL = 0         # channel for up/down eye movements

# Threshold parameters
HORIZONTAL_THRESHOLD = 3261.779
VERTICAL_THRESHOLD = 3188.374
BLINK_THRESHOLD = 4580.125

# Movement parameters
MOVE_STEP = 30               # Pixels to move per detection
DEAD_ZONE = 0.15             # Ignore small movements
BLINK_DURATION = 0.3         # Max blink duration (seconds)

# Screen boundaries (auto-detected)
SCREEN_WIDTH = None
SCREEN_HEIGHT = None

BUFFER_SIZE = 50