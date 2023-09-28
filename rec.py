"""
This script records video and data of a vehicle spawned in Carla. It assumes the JOAN Audi is spawned but may work with other vehicles as well. The script captures video of the vehicle as well as various sensor data such as acceleration, gyroscope readings, speed, steering angle, and GPS coordinates. The video and data are stored in .mp4 and .csv files, respectively. 

The script provides command line argument support for specifying the filename prefix for the output files, the directory to store the files, and an optional flag for recording an additonal wide field of view video.

Example usage:

    python3 rec.py filename /path/to/directory --wide

This will record wide video and store the output files in the specified directory with 'filename' as the prefix.
"""


import cv2
import csv
import carla
import numpy as np
import os
import argparse
from time import sleep
from datetime import datetime

from modules.carlainterface.carlainterface_process import connect_carla

# Constants
WIDTH, HEIGHT = 1928, 1208
FPS = 20
FILENAME = 'output'

class DataLogger:
    def __init__(self, csv_filename):
        self.file = open(csv_filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z',
                              'vehicle_speed', 'steer_angle', 'throttle', 'brake',
                              'gps_latitude', 'gps_longitude', 'gps_altitude'])

    def write_data(self, timestamp, accelerometer, gyroscope, speed, angle, gps):
        accel_x, accel_y, accel_z = (accelerometer.x, accelerometer.y, accelerometer.z) if accelerometer else (None, None, None)
        gyro_x, gyro_y, gyro_z = (gyroscope.x, gyroscope.y, gyroscope.z) if gyroscope else (None, None, None)
        lat, long, alt = (gps.latitude, gps.longitude, gps.altitude) if gps else (None, None, None)
        self.writer.writerow([timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, speed, angle, lat, long, alt])

    def close(self):
        self.file.close()


class VideoListener:
    def __init__(self, filename) -> None:
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_output = cv2.VideoWriter(filename, self.fourcc, FPS, (WIDTH,HEIGHT)) 

    def process_image(self, image):
        # Convert raw image data to numpy array
        img_array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        
        # Reshape image array to image dimensions and RGBA format
        img_array = np.reshape(img_array, (image.height, image.width, 4)) 
        img_array = cv2.cvtColor(img_array, cv2.COLOR_RGBA2RGB)

        if img_array is not None:
            self.video_output.write(img_array)
            # print('Hallo Frame')
            
        else:
            print("Image frame is empty.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help='filename prefix for videos and .csv')
    parser.add_argument('dir', nargs='?', default=os.getcwd(), help='directory to store the files' )
    parser.add_argument('--wide', action='store_true', help='record wide video')

    args = parser.parse_args()
    directory = os.path.normpath(args.dir)
    filepath = os.path.join(directory, datetime.now().strftime("%Y-%m-%d_%H-%M-%S_") + args.filename)

    _, _, world, _, _ = connect_carla()
    vehicle = world.get_actors().filter('vehicle.*')[0]  # get the first vehicle
    blueprint_library = world.get_blueprint_library()

    data_logger = DataLogger(filepath + '.csv')
    video_listener = VideoListener(filepath + '.mp4')       

    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(WIDTH))
    camera_bp.set_attribute('image_size_y', str(HEIGHT))
    camera_sensor = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=0., y=0., z=0.9), carla.Rotation()), attach_to=vehicle)

    if args.wide:
        video_listener_wide = VideoListener(filepath + '_wide.mp4')
        camera_bp_wide = blueprint_library.find('sensor.camera.rgb')
        camera_bp_wide.set_attribute('fov', '110')  # Setting a wide field of view
        camera_bp_wide.set_attribute('image_size_x', str(WIDTH))
        camera_bp_wide.set_attribute('image_size_y', str(HEIGHT))
        camera_sensor_wide = world.spawn_actor(camera_bp_wide, carla.Transform(carla.Location(x=0., y=0., z=0.9), carla.Rotation()), attach_to=vehicle)

    imu_bp = blueprint_library.find('sensor.other.imu')
    imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)

    gps_bp = blueprint_library.find('sensor.other.gnss')
    gps_sensor = world.spawn_actor(gps_bp, carla.Transform(), attach_to=vehicle)
    

    def update_vehicle_state(data, is_imu_data=True):
        speed = vehicle.get_velocity().length()
        angle = vehicle.get_transform().rotation.yaw

        # For IMU data
        if is_imu_data:
            data_logger.write_data(data.timestamp, data.accelerometer, data.gyroscope, speed, angle, None)

        # For GPS data
        else: 
            data_logger.write_data(data.timestamp, None, None, speed, angle, data)

    imu_sensor.listen(lambda data: update_vehicle_state(data))
    gps_sensor.listen(lambda data: update_vehicle_state(data, is_imu_data=False))
    camera_sensor.listen(video_listener.process_image)
    
    if args.wide:
        camera_sensor_wide.listen(video_listener_wide.process_image) 

    try:
        print('Recording started. Press Ctrl+C to terminate.')
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("\nRecording finished.")
    finally:
        imu_sensor.stop()
        imu_sensor.destroy()

        gps_sensor.stop()
        gps_sensor.destroy()

        camera_sensor.stop()
        camera_sensor.destroy()
        video_listener.video_output.release()
        
        if args.wide:
            camera_sensor_wide.stop()
            camera_sensor_wide.destroy()
            video_listener_wide.video_output.release()

        data_logger.close()


if __name__ == "__main__":
    main()