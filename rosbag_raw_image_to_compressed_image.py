#!/usr/bin/env python3

import os
import rosbag
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

def convert_and_save_images(input_bag_file, output_bag_file):
    # Open the input bag file for reading
    with rosbag.Bag(input_bag_file, 'r') as input_bag:
        # Create a new bag file for writing
        with rosbag.Bag(output_bag_file, 'w') as output_bag:
            bridge = CvBridge()
            for topic, msg, t in input_bag.read_messages():
                if topic == "/spinnaker_ros_driver_node/cam_fm_01/image_raw":  # Replace with your raw image topic
                    try:
                        # Convert the raw image to OpenCV format
                        raw_image_cv = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                        # Compress the image
                        _, compressed_image_data = cv2.imencode(".jpg", raw_image_cv)

                        # Create a new compressed image message
                        compressed_image_msg = CompressedImage()
                        compressed_image_msg.header = msg.header
                        compressed_image_msg.format = "jpeg"
                        compressed_image_msg.data = compressed_image_data.tobytes()

                        # Write the compressed image message to the output bag
                        output_bag.write("/spinnaker_ros_driver_node/cam_fm_01/compressed", compressed_image_msg, t)

                    except CvBridgeError as e:
                        print("CvBridge Error:", e)
                    except Exception as e:
                        print("Error:", e)
                else:
                    # For other topics, simply write the messages to the output bag
                    output_bag.write(topic, msg, t)


def process_all_bag_files(input_directory, output_directory):
    for root, dirs, files in os.walk(input_directory):
        for filename in files:
            if filename.endswith(".bag"):
                input_bag_file = os.path.join(root, filename)

                # Create subdirectories in the output directory if they don't exist
                relative_path = os.path.relpath(root, input_directory)
                output_subdirectory = os.path.join(output_directory, relative_path)
                os.makedirs(output_subdirectory, exist_ok=True)

                output_bag_file = os.path.join(output_subdirectory, filename)
                convert_and_save_images(input_bag_file, output_bag_file)

                info_output_file = os.path.splitext(output_bag_file)[0] + ".txt"
                os.system(f"rosbag info {input_bag_file} > {info_output_file}")


if __name__ == "__main__":
    input_directory = "/home/aesk/bags"  # Replace with your input directory path
    output_directory = "/home/aesk/bags_compressed"  # Replace with your output directory path

    process_all_bag_files(input_directory, output_directory)
