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
WIDTH, HEIGHT = 1280,720 #1928, 1208

X_CAM, Y_CAM, Z_CAM = 1.5, 0, 0.9
X_CAM_WIDE, Y_CAM_WIDE, Z_CAM_WIDE = 1.6, 0, 0.9
FOV_WIDE = 145

class DataLogger:
    def __init__(self, csv_filename):
        self.file = open(csv_filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z',
                              'vehicle_speed', 'yaw', 'throttle', 'brake', 'steer',
                              'position_x', 'position_y', 'position_z'])

    def write_data(self, timestamp, accelerometer, gyroscope, speed, control, yaw, position):
        accel_x, accel_y, accel_z = (accelerometer.x, accelerometer.y, accelerometer.z) if accelerometer else (None, None, None)
        gyro_x, gyro_y, gyro_z = (gyroscope.x, gyroscope.y, gyroscope.z) if gyroscope else (None, None, None)
        pos_x, pos_y, pos_z, = (position.x, position.y, position.z) if position else (None, None, None)
        throttle, brake, steer = (control.throttle, control.brake, control.steer) if control else (None, None, None)

        self.writer.writerow([timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, speed, yaw, throttle, brake, steer, pos_x, pos_y, pos_z])

    def close(self):
        self.file.close()

class VideoListener:
    def __init__(self, filename, fps) -> None:
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_output = cv2.VideoWriter(filename, self.fourcc, fps, (WIDTH,HEIGHT)) 

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
    parser.add_argument('--fps', default=20, type=int, help='frames per second')
    parser.add_argument('--timestamp', action='store_true', help='prepend file with timestamp')

    args = parser.parse_args()
    directory = os.path.normpath(args.dir)

    if args.timestamp:
        filepath = os.path.join(directory, datetime.now().strftime("%Y-%m-%d_%H-%M-%S_") + args.filename)
    else:
        filepath = os.path.join(directory, args.filename)

    _, _, world, _, _ = connect_carla()

    vehicle = world.get_actors().filter('vehicle.*')[0]  # get the first vehicle
    blueprint_library = world.get_blueprint_library()

    data_logger = DataLogger(filepath + '.csv')

    video_listener = VideoListener(filepath + '.mp4', args.fps)
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(WIDTH))
    camera_bp.set_attribute('image_size_y', str(HEIGHT))   

    if args.wide:
        video_listener_wide = VideoListener(filepath + '_wide.mp4', args.fps) 
        camera_bp_wide = blueprint_library.find('sensor.camera.rgb')
        camera_bp_wide.set_attribute('fov', str(FOV_WIDE))  # Setting a wide field of view
        camera_bp_wide.set_attribute('image_size_x', str(WIDTH))
        camera_bp_wide.set_attribute('image_size_y', str(HEIGHT))   



    camera_sensor = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=X_CAM, y=Y_CAM, z=Z_CAM), carla.Rotation()), attach_to=vehicle)
    if args.wide:
        camera_sensor_wide = world.spawn_actor(camera_bp_wide, carla.Transform(carla.Location(x=X_CAM_WIDE, y=Y_CAM_WIDE, z=Z_CAM_WIDE), carla.Rotation()), attach_to=vehicle)

    imu_bp = blueprint_library.find('sensor.other.imu')
    imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)

    def update_vehicle_state(data):
        speed = vehicle.get_velocity().length()
        yaw = vehicle.get_transform().rotation.yaw
        position = vehicle.get_transform().location
        control = vehicle.get_control()
       
        data_logger.write_data(data.timestamp, data.accelerometer, data.gyroscope, speed, control, yaw, position)

    imu_sensor.listen(lambda data: update_vehicle_state(data))
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
        data_logger.close()

        camera_sensor.stop()
        if args.wide:
            camera_sensor_wide.stop()

        camera_sensor.destroy()
        if args.wide:
            camera_sensor_wide.destroy()
            
        video_listener.video_output.release()
        if args.wide:
            video_listener_wide.video_output.release()

if __name__ == "__main__":
    main()