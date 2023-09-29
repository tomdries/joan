"""
Usage:
    python video_stretch.py path_to_video path_to_data 
"""

import numpy as np
import subprocess
import pandas as pd 
import sys

def mp4_duration(video_path):
    command = ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "default=noprint_wrappers=1:nokey=1", video_path]
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return float(result.stdout)

def stretch_video(video_path, timestamp_array):
    # get video duration (mp4):
    original_duration = mp4_duration(video_path)
    
    # Calculate desired video duration based on timestamps
    desired_duration = timestamp_array[-1] - timestamp_array[0]

    # Calculate speed factor
    speed_factor = desired_duration / original_duration 


    # Prepare output file path
    output_video_path = video_path.replace(".mp4", "_stretched.mp4")
    print(video_path)
    # Use ffmpeg to adjust video speed
    command = f"ffmpeg -i {video_path} -filter:v 'setpts={speed_factor}*PTS' {output_video_path}"
    command = f"ffmpeg -i {video_path} -filter:v setpts={speed_factor}*PTS {output_video_path}"

    print(command)# Execute the command to adjust video speed
    print(f"Adjusting video length (factor {speed_factor:.4f}. \nDesired duration: {desired_duration} s")
    subprocess.call(command, shell=True)
    print("Done")

    return output_video_path


def test_constant_fps(timestamp_array, tolerance=0.01, showstats=False):
    """
    Test if the frame rate is approximately constant.

    Parameters:
    timestamp_array (list): List of timestamps for each frame in the video.
    tolerance (float): Tolerance for variation in frame duration. Default is 0.01 seconds.

    Returns:
    bool: True if the frame rate is approximately constant, False otherwise.
    """
    
    dur = np.diff(timestamp_array) # durations
    stats_str = f"(M = {dur.mean():.2f}, sd = {np.std(dur):.2f}, max = {max(dur):.2f}, min = {min(dur):.2f})" 
    # Check if the standard deviation is below the tolerance
    if np.std(dur) < tolerance:
        print(f"The duration between frames is approximately constant. " + stats_str)
        return True
    else:
        print(f"The duration between frames varies significantly. " + stats_str)
        return False    
    
if __name__ == '__main__':
    video_path = sys.argv[1]
    data_path = sys.argv[2]
    timestamps = pd.read_csv(data_path, usecols=['timestamp']).values.flatten()
    assert test_constant_fps(timestamps)
    stretch_video(video_path, timestamps)







