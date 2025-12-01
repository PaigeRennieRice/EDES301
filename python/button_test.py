#!/usr/bin/env python3
# --------------------------------------------------------------------------
# Button Test Script
# --------------------------------------------------------------------------
# Quick script to verify wiring + library functionality for BUTTON.Button
# LED optional (uncomment if you want visual confirmation)
# --------------------------------------------------------------------------

import time
import button as BUTTON
# import led as LED     # optional — if you want an LED indicator

BUTTON_PIN = "P2_2"     # Change if needed
# LED_PIN    = "P2_6"   # optional

def main():
    print("Starting Button Test...")
    print("Press the button to see events printed here.")
    print("Press CTRL+C to exit.\n")

    btn = BUTTON.Button(BUTTON_PIN)
    # led = LED.LED(LED_PIN)  # optional

    while True:
        if btn.is_pressed():
            press_time = time.time()

            # Optional LED indicator
            # led.on()

            print("Button pressed!")

            # Wait for release
            while btn.is_pressed():
                time.sleep(0.01)

            release_time = time.time()
            held_duration = release_time - press_time

            # Optional LED indicator
            # led.off()

            print(f"Button released. Held for {held_duration:.3f} seconds.\n")

        time.sleep(0.01)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting button test...")
