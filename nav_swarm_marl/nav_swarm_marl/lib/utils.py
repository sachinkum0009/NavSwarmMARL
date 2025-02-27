from sensor_msgs.msg import LaserScan
import numpy as np

def filter_laserscan(input: LaserScan) -> LaserScan:
    total_rays = len(input.ranges)
    print(f"Total rays: {total_rays}")

    if total_rays == 0:
        # self.get_logger().warn("Received empty LaserScan data")
        return

    # Compute correct indices for 270° to 90° range
    start_index = (3 * total_rays) // 4  # 270 degrees
    end_index = total_rays // 4   # 90 degrees

    # Extract data for the 270° → 90° sector (handling wrap-around)
    if start_index > end_index:
        mid_ranges = input.ranges[start_index:] + input.ranges[:end_index + 1]
    else:
        mid_ranges = input.ranges[start_index:end_index + 1]

    # Select 61 rays evenly spaced from these 180 rays
    selected_indices = np.linspace(0, len(mid_ranges) - 1, 61, dtype=int)
    filtered_ranges = [mid_ranges[i] for i in selected_indices]

    # Compute new angle_min and angle_max correctly (270° to 90°)
    new_angle_min = -1.571 #input.angle_min + start_index * input.angle_increment
    new_angle_max = 1.571 #input.angle_min + end_index * input.angle_increment

    # Create filtered LaserScan message
    filtered_scan = LaserScan()
    filtered_scan.header = input.header
    filtered_scan.angle_min = new_angle_min
    filtered_scan.angle_max = new_angle_max
    filtered_scan.angle_increment = (new_angle_max - new_angle_min) / (len(filtered_ranges) - 1)
    filtered_scan.time_increment = input.time_increment
    filtered_scan.scan_time = input.scan_time
    filtered_scan.range_min = input.range_min
    filtered_scan.range_max = input.range_max
    filtered_scan.ranges = filtered_ranges
    # filtered_scan.intensities = [input.intensities[start_index + i] for i in selected_indices] if input.intensities else []


    # Extract 180-degree sector (middle half of 360-degree scan)
    # start_index = total_rays // 4   # 90 degrees
    # end_index = (3 * total_rays) // 4  # 270 degrees
    # mid_ranges = input.ranges[start_index:end_index]

    # # Select 61 rays evenly spaced from these 180 rays
    # selected_indices = np.linspace(0, len(mid_ranges) - 1, 61, dtype=int)
    # filtered_ranges = [mid_ranges[i] for i in selected_indices]

    # # Compute new angle_min and angle_max
    # new_angle_min = input.angle_min + start_index * input.angle_increment
    # new_angle_max = input.angle_min + end_index * input.angle_increment

    # # Create filtered LaserScan message
    # filtered_scan = LaserScan()
    # filtered_scan.header = input.header
    # filtered_scan.angle_min = new_angle_min
    # filtered_scan.angle_max = new_angle_max
    # filtered_scan.angle_increment = (new_angle_max - new_angle_min) / (len(filtered_ranges) - 1)
    # filtered_scan.time_increment = input.time_increment
    # filtered_scan.scan_time = input.scan_time
    # filtered_scan.range_min = input.range_min
    # filtered_scan.range_max = input.range_max
    # filtered_scan.ranges = filtered_ranges
    # filtered_scan.intensities = [input.intensities[start_index + i] for i in selected_indices] if input.intensities else []

    # Assuming 360-degree scan, get the middle 180 degrees
    # start_index = total_rays // 4   # 90 degrees
    # end_index = (3 * total_rays) // 4  # 270 degrees

    # filtered_scan = LaserScan()
    # filtered_scan.header = input.header
    # filtered_scan.angle_min = input.angle_min + start_index * input.angle_increment
    # filtered_scan.angle_max = input.angle_min + end_index * input.angle_increment
    # filtered_scan.angle_increment = input.angle_increment
    # filtered_scan.time_increment = input.time_increment
    # filtered_scan.scan_time = input.scan_time
    # filtered_scan.range_min = input.range_min
    # filtered_scan.range_max = input.range_max
    # filtered_scan.ranges = input.ranges[start_index:end_index]
    # filtered_scan.intensities = input.intensities[start_index:end_index] if input.intensities else []
    # angle_increment = input.angle_increment
    
    # # Calculate the number of rays corresponding to 180 degrees
    # num_rays_180 = int((3.14159) / angle_increment)  # 180 degrees in radians
    
    # if num_rays_180 > total_rays:
    #     print("Not enough rays to extract 180 degrees of data.")
    #     return None
    
    # # Select 61 evenly spaced rays within 180 degrees
    # step = max(1, num_rays_180 // 61)
    # selected_ranges = input.ranges[:num_rays_180:step]
    # selected_intensities = input.intensities[:num_rays_180:step] if input.intensities else []
    
    # # Create a new LaserScan message
    # filtered_scan = LaserScan()
    # filtered_scan.header = input.header
    # filtered_scan.angle_min = input.angle_min
    # filtered_scan.angle_max = input.angle_min + (len(selected_ranges) - 1) * angle_increment * step
    # filtered_scan.angle_increment = angle_increment * step
    # filtered_scan.time_increment = input.time_increment * step
    # filtered_scan.scan_time = input.scan_time
    # filtered_scan.range_min = input.range_min
    # filtered_scan.range_max = input.range_max
    # filtered_scan.ranges = selected_ranges
    # filtered_scan.intensities = selected_intensities
    
    # print("Published filtered LaserScan with {} rays".format(len(filtered_scan.ranges)))
    return filtered_scan
