from sensor_msgs.msg import LaserScan

def filter_laserscan(input: LaserScan) -> LaserScan:
    total_rays = len(input.ranges)
    angle_increment = input.angle_increment
    
    # Calculate the number of rays corresponding to 180 degrees
    num_rays_180 = int((3.14159) / angle_increment)  # 180 degrees in radians
    
    if num_rays_180 > total_rays:
        print("Not enough rays to extract 180 degrees of data.")
        return None
    
    # Select 61 evenly spaced rays within 180 degrees
    step = max(1, num_rays_180 // 61)
    selected_ranges = input.ranges[:num_rays_180:step]
    selected_intensities = input.intensities[:num_rays_180:step] if input.intensities else []
    
    # Create a new LaserScan message
    filtered_scan = LaserScan()
    filtered_scan.header = input.header
    filtered_scan.angle_min = input.angle_min
    filtered_scan.angle_max = input.angle_min + (len(selected_ranges) - 1) * angle_increment * step
    filtered_scan.angle_increment = angle_increment * step
    filtered_scan.time_increment = input.time_increment * step
    filtered_scan.scan_time = input.scan_time
    filtered_scan.range_min = input.range_min
    filtered_scan.range_max = input.range_max
    filtered_scan.ranges = selected_ranges
    filtered_scan.intensities = selected_intensities
    
    print("Published filtered LaserScan with {} rays".format(len(selected_ranges)))
    return filtered_scan