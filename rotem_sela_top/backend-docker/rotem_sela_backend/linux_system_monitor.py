import os
import subprocess
import usb.core
import usb.util
import v4l2py
import re

class LinuxSystemStatus:
    @classmethod
    def get_list_of_video_devices(cls, bus_num, dev_address):
        """Given a bus number and device address, returns the /dev/videoX path."""
        video_devices_base = '/sys/class/video4linux'
        matches = []
        for video_dev in os.listdir(video_devices_base):
            video_device_path = os.path.join(video_devices_base, video_dev)
            if os.path.islink(video_device_path):
                symlink_path = os.readlink(video_device_path)
                match_bus = f"usb{bus_num}"
                if match_bus in symlink_path:
                    parent_path = os.path.dirname(os.path.dirname(os.path.dirname(symlink_path)))
                    devnum_path = os.path.join(video_devices_base, parent_path, 'devnum')
                    try:
                        with open(devnum_path, 'r') as f:
                            devnum_value = int(f.read().strip())
                            if devnum_value == dev_address:
                                matches.append("/dev/{}".format(video_dev))
                    except FileNotFoundError:
                        pass  # handle it accordingly
        
        return matches
    
    @classmethod
    def is_device_operational(cls, device_path):
        try:
            with v4l2py.Device(device_path) as device:
                caps = device.info.capabilities
                if caps:
                    return True
        except Exception as e:
            pass
        return False


    @classmethod
    def get_working_video_device(cls, list_of_devices):
        for device in list_of_devices:
            if LinuxSystemStatus.is_device_operational(device):
                return device
        return "None"
        
    @classmethod
    def list_usb_cameras(cls):
        try:
            # Use subprocess.run for simplicity
            result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True, check=False)

            # Split the output by lines
            lines = result.stdout.split('\n')

            # Initialize a list to store camera information
            camera_list = []

            # Initialize variables to track port and whether we've found a video device for the current camera
            current_port = None
            found_video_device = False

            # Iterate through lines to extract relevant information
            for line in lines:
                # Extract the port information
                port_match = re.search(r'usb-[^:]+:([^)]+)', line)
                if port_match:
                    current_port = port_match.group(1).strip()
                    found_video_device = False  # Reset the flag for each camera

                elif '/dev/video' in line and not found_video_device:
                    # Extract the video device information
                    video_device = line.split(':')[-1].strip()

                    # Check if both port and device are available
                    if current_port:
                        # Append to the camera list
                        camera_list.append([current_port, video_device])
                        found_video_device = True  # Set the flag to True once we find a video device

            return camera_list

        except subprocess.CalledProcessError as e:
            # Handle the case where v4l2-ctl returns a non-zero exit code
            print(f"Error with v4l2-ctl --list-devices: {e.stderr}")
            return []
        except FileNotFoundError:
            # Handle the case where v4l2-ctl is not found
            print("Error: v4l2-ctl not found. Make sure it is installed.")
            return []
        except Exception as e:
            print(f"Exception with v4l2-ctl --list-devices: {e}")
            return []

if __name__ == "__main__":
    cameras = LinuxSystemStatus.list_usb_cameras()
    if cameras:
        print(cameras)
    else:
        print("Error getting USB camera information.")

