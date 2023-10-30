import os

def get_bus_info_for_video_device(device):
    device_path = f"/sys/class/video4linux/{device}"
    
    # Fetch the symlink target, which contains the bus info
    try:
        symlink_target = os.readlink(device_path)
        print(f"symlink_target={symlink_target}")
        parts = symlink_target.split('/')
        pci_info = [part for part in parts if part.startswith("pci")][0]
        usb_parts = [part for part in parts if part.startswith("usb") or part.startswith('1-')]
        bus_info = f"usb-{pci_info}-{'.'.join(usb_parts)}"
        return bus_info    
    except OSError:
        print(f"Error reading symlink for {device}")
        return None

def main():
    base_path = '/sys/class/video4linux'
    video_devices = os.listdir(base_path)

    for video_device in video_devices:
        bus_info = get_bus_info_for_video_device(video_device)
        if bus_info:
            print(f"{video_device} -> {bus_info}")
        else:
            print(f"Could not retrieve bus info for {video_device}")

if __name__ == "__main__":
    main()

