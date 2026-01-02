import time
import board
import busio
import adafruit_vl53l0x

def test_sensor():
    try:
        # 1. Initialize I2C bus (SDA/SCL pins)
        i2c = busio.I2C(board.SCL, board.SDA)

        # 2. Initialize VL53L0X sensor
        # Default I2C address is 0x29
        sensor = adafruit_vl53l0x.VL53L0X(i2c)

        print("--- VL53L0X Sensor Initialized Successfully ---")
        print("Reading distance... Press Ctrl+C to stop.")

        while True:
            # 3. Read distance in millimeters (mm)
            distance_mm = sensor.range
            
            # 4. Convert to meters (m) for your PID loop
            distance_m = distance_mm / 1000.0

            # Output the distance
            # Note: 8190 or 8191 usually means 'Out of Range'
            if distance_mm > 8000:
                print("Distance: Out of Range")
            else:
                print(f"Distance: {distance_m:.3f} meters ({distance_mm} mm)")

            time.sleep(0.3)  # 10Hz update rate for testing

    except ValueError as e:
        print(f"Hardware Error: {e}")
        print("Check your wiring (SDA/SCL pins) and ensure I2C is enabled in raspi-config.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    test_sensor()



