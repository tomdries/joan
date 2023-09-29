import matplotlib.pyplot as plt
import cv2
import numpy as np

def load_video_frame(path, frame_nr):
    video = cv2.VideoCapture(path)
    video.set(cv2.CAP_PROP_POS_FRAMES, frame_nr)
    ret, frame = video.read()
    if ret:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        plt.imshow(frame)
        plt.show()
    else:
        print(f"Failed to load frame {frame_nr} from {path}")
    video.release()


def count_total_frames(path):
    '''get total amount of frames'''
    video = cv2.VideoCapture(path)
    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    video.release()
    return total_frames

def xy_to_gps(x, y, x_origin, y_origin):
    """Convert x, y coordinates to GPS coordinates (spherical approximation)."""
    # Earth’s radius, sphere
    R=6378137

    # offsets in meters
    dn = y
    de = x

    # Coordinate offsets in radians
    dLat = dn/R
    dLon = de/(R*np.cos(np.pi*x_origin/180))

    # OffsetPosition, decimal degrees
    lat = dLat * 180/np.pi
    long = dLon * 180/np.pi

    lat += y_origin
    long += x_origin

    return lat, long