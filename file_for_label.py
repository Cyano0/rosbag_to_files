import os
import sys
import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, PointCloud2
from scipy.io import savemat
import struct
import open3d as o3d

class BagDataExtractor:
    def __init__(self, bag_file, base_output_dir):
        self.bag_file = bag_file
        self.base_output_dir = os.path.abspath(base_output_dir)

        # Define topics and output directories
        self.image_topics = {
            '/front_camera/image_raw': os.path.join(self.base_output_dir, 'output_images'),
            '/fisheye_image_SN00013': os.path.join(self.base_output_dir, 'fisheye_images_13'),
            '/fisheye_image_SN00012': os.path.join(self.base_output_dir, 'fisheye_images_12'),
            '/fisheye_image_SN00014': os.path.join(self.base_output_dir, 'fisheye_images_14')
        }

        self.lidar_topic = '/front_lidar/points'
        self.lidar_output_dir = os.path.join(self.base_output_dir, 'lidar_points')

        # Create directories
        self.timestamps = {topic: [] for topic in self.image_topics}
        self.timestamps[self.lidar_topic] = []  
        os.makedirs(self.lidar_output_dir, exist_ok=True)

        for folder in self.image_topics.values():
            os.makedirs(folder, exist_ok=True)

    def extract_data(self):
        """ Reads the ROS2 bag file and extracts images and LiDAR data """
        print(f"Reading bag file: {self.bag_file}")

        # Open the bag file
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_file, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
        reader.open(storage_options, converter_options)

        # Get available topics and their types
        topic_types = reader.get_all_topics_and_types()
        topic_type_dict = {t.name: t.type for t in topic_types}

        # Filter topics (images and LiDAR)
        valid_image_topics = {t: topic_type_dict[t] for t in self.image_topics if t in topic_type_dict}
        lidar_available = self.lidar_topic in topic_type_dict

        # Get message types
        msg_types = {topic: get_message(msg_type) for topic, msg_type in valid_image_topics.items()}
        if lidar_available:
            msg_types[self.lidar_topic] = get_message(topic_type_dict[self.lidar_topic])

        # Read messages
        count = {topic: 0 for topic in valid_image_topics}
        lidar_count = 0

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic in valid_image_topics:
                self.extract_image(topic, data, msg_types[topic], count)

            elif topic == self.lidar_topic and lidar_available:
                self.extract_lidar(data, msg_types[self.lidar_topic], lidar_count)
                lidar_count += 1

        # Save timestamps
        self.save_timestamps()
        print("Data extraction completed!")

    def extract_image(self, topic, data, msg_type, count):
        """ Extracts and saves image from ROS2 bag """
        msg = deserialize_message(data, msg_type)
        try:
            cv_image = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

            # Generate timestamp-based filename
            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            timestamp_float = timestamp_sec + timestamp_nanosec * 1e-9
            timestamp_str = f"{timestamp_sec}_{timestamp_nanosec:09d}"
            output_file = os.path.join(self.image_topics[topic], f"{timestamp_str}.png")

            # Save image
            cv2.imwrite(output_file, cv_image)
            count[topic] += 1
            self.timestamps[topic].append(timestamp_float)

            print(f"Saved image: {output_file}")

        except Exception as e:
            print(f"Error processing image from {topic}: {e}")

    def extract_lidar(self, data, msg_type, count):
        """ Extracts and saves LiDAR PointCloud2 data from ROS2 bag as .bin file """
        msg = deserialize_message(data, msg_type)
        try:
            points = self.convert_pointcloud2_to_numpy(msg)

            # Generate timestamp-based filename
            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            timestamp_str = f"{timestamp_sec}_{timestamp_nanosec:09d}"
            
            timestamp_float = timestamp_sec + timestamp_nanosec * 1e-9
            self.timestamps[self.lidar_topic].append(timestamp_float)

            # # Save to .bin file
            # points.tofile(output_file)
            # print(f"Saved LiDAR point cloud: {output_file}")
            # Convert to Nx3 or Nx4 array
            xyz = np.vstack((points['x'], points['y'], points['z'])).T
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)

            # Optional: add intensity as color
            if 'intensity' in points.dtype.names:
                intensities = points['intensity']
                intensities_normalized = np.clip(intensities / np.max(intensities), 0, 1)
                colors = np.tile(intensities_normalized.reshape(-1, 1), (1, 3))  # Grayscale
                pcd.colors = o3d.utility.Vector3dVector(colors)

            output_file = os.path.join(self.lidar_output_dir, f"{timestamp_str}.pcd")
            o3d.io.write_point_cloud(output_file, pcd)
            print(f"✅ Saved LiDAR point cloud: {output_file}")

        except Exception as e:
            print(f"Error processing LiDAR data: {e}")

    def convert_pointcloud2_to_numpy(self, msg):
        """ Converts ROS2 PointCloud2 message to a NumPy array in KITTI format """
        assert isinstance(msg, PointCloud2), "Message is not of type sensor_msgs/PointCloud2"

        field_names = [field.name for field in msg.fields]
        point_step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, point_step)

        x_idx = field_names.index("x")
        y_idx = field_names.index("y")
        z_idx = field_names.index("z")
        intensity_idx = field_names.index("intensity") if "intensity" in field_names else None

        x_offset = msg.fields[x_idx].offset
        y_offset = msg.fields[y_idx].offset
        z_offset = msg.fields[z_idx].offset
        intensity_offset = msg.fields[intensity_idx].offset if intensity_idx is not None else None

        dtype_list = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32) if intensity_offset is not None else ('intensity', np.float32)
        ]

        points = np.zeros(len(data), dtype=dtype_list)

        for i, row in enumerate(data):
            points[i]['x'] = struct.unpack('f', row[x_offset:x_offset+4])[0]
            points[i]['y'] = struct.unpack('f', row[y_offset:y_offset+4])[0]
            points[i]['z'] = struct.unpack('f', row[z_offset:z_offset+4])[0]
            if intensity_offset is not None:
                points[i]['intensity'] = struct.unpack('f', row[intensity_offset:intensity_offset+4])[0]

        return points

    def save_timestamps(self):
        """ Saves timestamps as MATLAB-compatible .mat files """
        for topic, timestamps in self.timestamps.items():
            if topic in self.image_topics:
                output_dir = self.image_topics[topic]
            elif topic == self.lidar_topic:
                output_dir = self.lidar_output_dir
            else:
                print(f"⚠️ Unknown topic {topic}, skipping timestamp save.")
                continue

            timestamp_file = os.path.join(output_dir, "timestamps.mat")
            timestamps_np = np.array(timestamps).reshape(-1, 1)
            savemat(timestamp_file, {"timestamps": timestamps_np})
            print(f"✅ Saved timestamps: {timestamp_file}")



if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python extract_data.py <bag_file> <output_directory>")
        sys.exit(1)

    bag_file = sys.argv[1]
    output_dir = sys.argv[2]

    extractor = BagDataExtractor(bag_file, output_dir)
    extractor.extract_data()

