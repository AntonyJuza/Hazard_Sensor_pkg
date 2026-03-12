import Adafruit_DHT
import time

sensor = Adafruit_DHT.DHT22
pin = 27  # GPIO4

while True:
    humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

    if humidity is not None and temperature is not None:
        print(f"Temp: {temperature:.1f}°C  Humidity: {humidity:.1f}%")
    else:
        print("Failed to retrieve data")

    time.sleep(2)