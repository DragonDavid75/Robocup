#!/usr/bin/env python3

import time
from mqtt_python.sgpio import gpio

def main():
    gpio.setup()

    print("[TEST] Testing stop button on BCM GPIO 6")
    print("[TEST] Press Ctrl+C to exit")

    try:
        while True:
            if gpio.test_stop_button():
                print("[TEST] STOP button pressed!")
            else:
                print("[TEST] not pressed")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[TEST] exiting")

    finally:
        gpio.terminate()

if __name__ == "__main__":
    main()
