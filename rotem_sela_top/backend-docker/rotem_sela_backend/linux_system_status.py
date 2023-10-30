import os

import usb.core
import usb.util
import v4l2py


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
                                matches.append(video_dev)
                    except FileNotFoundError:
                        pass  # handle it accordingly
        return matches

    @classmethod
    def is_device_operational(cls, video_device):
        try:
            device_path = f"/dev/{video_device}"
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
        # Find all devices
        devices = usb.core.find(find_all=True)
        camera_list = []

        for dev in devices:
            try:
                # Get the details of the device
                iProduct = dev.product
                if "camera" in iProduct.lower():
                    serial = f"{dev.idVendor}-{dev.idProduct}-{dev.bcdDevice}"
                    dev_address = dev.address
                    matched_video_devices = LinuxSystemStatus.get_list_of_video_devices(bus_num=dev.bus,
                                                                                        dev_address=dev_address)
                    video_device = LinuxSystemStatus.get_working_video_device(list_of_devices=matched_video_devices)
                    camera_list.append({
                        "name": iProduct,
                        "serial": serial,
                        "video": video_device
                    })
            except Exception as e:
                print(f"Couldn't get info for device due to {e}")

        return camera_list


if __name__ == "__main__":
    cameras = LinuxSystemStatus.list_usb_cameras()
    for camera in cameras:
        print(camera)
