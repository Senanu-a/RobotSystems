#!/usr/bin/env python3
from time import sleep
from picarx_improved import Picarx

px = Picarx()

print("Reading grayscale sensors (Ctrl+C to stop)")
print("Format: RAW=[L, M, R]  STATE=[L, M, R]")
print("-" * 50)

try:
    while True:
        # Read raw analog values (approx voltages)
        raw_vals = px.get_grayscale_data()

        # Convert to binary line/background
        state_vals = px.get_line_status(raw_vals)

        print(f"RAW={raw_vals}  STATE={state_vals}")

        sleep(0.2)

except KeyboardInterrupt:
    print("\nStopping sensor read...")

finally:
    px.stop()
    sleep(0.1)
    print("Done.")
