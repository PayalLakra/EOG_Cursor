import time
from pylsl import StreamInlet, resolve_stream
import csv
from datetime import datetime

STREAM_NAME = "NPG"
STREAM_TYPE = "EXG"
SOURCE_ID = "npg1234"

RECORD_SECONDS = 10.0
FILENAME = f"up.csv"

def record_eog():
    streams = [s for s in resolve_stream() 
              if s.name() == STREAM_NAME 
              and s.type() == STREAM_TYPE
              and s.source_id() == SOURCE_ID]
    
    if not streams:
        raise RuntimeError(f"Stream '{STREAM_NAME}' ({STREAM_TYPE}) not found")

    inlet = StreamInlet(streams[0])
    print(f"Connected to: {streams[0].name()} (Channels: {streams[0].channel_count()}, Rate: {streams[0].nominal_srate()}Hz)")

    with open(FILENAME, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Counter', 'Channel1', 'Channel2'])
        start_time = time.time()
        counter = 0

        try:
            while RECORD_SECONDS is None or (time.time() - start_time) < RECORD_SECONDS:
                sample, _ = inlet.pull_sample()
                writer.writerow([counter, sample[0], sample[1]])
                counter += 1
                
                if counter % int(inlet.info().nominal_srate()) == 0:
                    print(f"Collected {counter} samples...")

        except KeyboardInterrupt:
            print("Stopped by user")
        
        duration = time.time() - start_time

if __name__ == "__main__":
    record_eog()