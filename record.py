#!/usr/bin/python3
import threading
import picamera
import struct
import queue
import smbus
import gpsd
import time
import json
import sys
import os

COMPASS_ADDR = 0x1e
ACCELEROMETER_ADDR = 0x53
GYROSCOPE_ADDR = 0x68

log_queue = queue.Queue(maxsize=4096)

def logger():
  while True:
    with open("./data/mission.log", "a+") as LOG:
      while not log_queue.empty():
        try:
          LOG.write(json.dumps(log_queue.get())+"\n")
        except Exception as e:
          print("Failed to log: {}".format(e))
          break

def log(data):
  log_queue.put(data)

def record_video():
  with picamera.PiCamera() as camera:
    camera.resolution = (1920, 1080)
    camera.framerate = 30
    num = 0
    while os.path.exists('data/vid{:03d}.h264'.format(num)):
      num += 1
    print("Recording video to data/vid{:03d}.h264".format(num))
    camera.start_recording('data/vid{:03d}.h264'.format(num))
  
    while True:
      frame = camera.frame
      data = {
        "source": "video",
        "index": frame.index,
        "frame_type": frame.frame_type,
        "frame_timestamp": frame.timestamp,
        "clock_timestamp": time.time(),
        "complete": frame.complete,
        "position": frame.position
      }
      log(data)
      camera.wait_recording(0.4/camera.framerate)
    camera.stop_recording()

def record_gps():
  gpsd.connect()
  while True:
    data = gpsd.get_current()
    log({
      "source": "gps",
      "clock_timestamp": time.time(),
      "lat": data.lat,
      "lon": data.lon,
      "alt": data.alt,
      "hspeed": data.hspeed,
      "track": data.track,
      "climb": data.climb,
      "error": data.error,
      "time": data.time,
      "mode": data.mode,
      "sats": data.sats,
    })
    time.sleep(0.1)

def record_imu():
  bus = smbus.SMBus(1)
  # Datasheet: http://www.soc-robotics.com/pdfs/HMC5883L.pdf
  compass_config = [
    (0x00, 0b01111000), #Config reg A, 8xavg, 75Hz output rate, normal bias
    (0x01, 0b00100000), #Config reg B, +/- 1.3 Ga
    (0x02, 0b00000000), #Mode reg, continuous mode
  ]
  for i in compass_config:
    bus.write_byte_data(COMPASS_ADDR, *i)
  while True:
    status = bus.read_byte_data(COMPASS_ADDR, 0x09)
    if status % 2:
      #Compass reading is ready
      data = []
      for i in range(6):
        data.append(bus.read_byte_data(COMPASS_ADDR, 0x03+i)) #0x03 is start of data registers
      data = bytes(data)
      x = struct.unpack('>h', data[:2])
      y = struct.unpack('>h', data[2:4])
      z = struct.unpack('>h', data[4:6])
      log({
        "source": "compass",
        "clock_timestamp": time.time(),
        "x": x,
        "y": y,
        "z": z,
      })

logger_thread = threading.Thread(target=logger)
logger_thread.start()
video_thread = threading.Thread(target=record_video)
video_thread.start()
gps_thread = threading.Thread(target=record_gps)
gps_thread.start()
imu_thread = threading.Thread(target=record_imu)
imu_thread.start()
print("Capturing data...")
