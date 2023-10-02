#!/usr/bin/env python3

import os
import yaml
import shutil
import rosbag
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

def convert_and_save_images(input_bag_file, output_bag_file, config):
    with rosbag.Bag(input_bag_file, 'r') as input_bag:
        
        is_there_raw_image_in_bag = False
        is_there_raw_image_in_bag = any(types_and_topic_info.msg_type == "sensor_msgs/Image"
                                        for types_and_topic_info in input_bag.get_type_and_topic_info()[1].values())

        if not is_there_raw_image_in_bag and config["copy"]:
            shutil.copyfile(input_bag_file, output_bag_file)
            return True
        
        if is_there_raw_image_in_bag:
            with rosbag.Bag(output_bag_file, 'w') as output_bag:
                bridge = CvBridge()
                for topic, msg, t in input_bag.read_messages():
                    if msg._type == "sensor_msgs/Image":
                        try:
                            raw_image_cv = bridge.imgmsg_to_cv2(
                                msg, desired_encoding="passthrough")

                            _, compressed_image_data = cv2.imencode(".jpg", raw_image_cv)

                            compressed_image_msg = CompressedImage()
                            compressed_image_msg.header = msg.header
                            compressed_image_msg.format = "jpeg"
                            compressed_image_msg.data = compressed_image_data.tobytes()

                            topic = topic.replace("/image_raw", "")
                            output_bag.write(topic + "/compressed", compressed_image_msg, t)
                            progress += 1

                        except CvBridgeError as e:
                            print("CvBridge Error:", e)
                        except Exception as e:
                            print("Error:", e)
                    else:
                        output_bag.write(topic, msg, t)
                return True
        else:
            return False


def process_all_bag_files(input_directory, output_directory, config):
    processed_files = []
    for root, dirs, files in os.walk(input_directory):
        for filename in files:
            if filename.endswith(".bag"):

                input_bag_file = os.path.join(root, filename)

                relative_path = os.path.relpath(root, input_directory)
                output_subdirectory = os.path.join(
                    output_directory, relative_path)
                os.makedirs(output_subdirectory, exist_ok=True)

                output_bag_file = os.path.join(output_subdirectory, filename)
                processed = convert_and_save_images(
                    input_bag_file, output_bag_file, config)

                if processed:
                    processed_files.append(input_bag_file)
            
                if config["info"] and processed:
                    info_output_file = os.path.splitext(
                        output_bag_file)[0] + ".txt"
                    
                    # due to >>> sh: 1: Syntax error: "(" unexpected
                    output_bag_file_quoted = "\"" + output_bag_file + "\""
                    info_output_file_quoted = "\"" + info_output_file + "\""
                    os.system("rosbag info " + output_bag_file_quoted + " > " + info_output_file_quoted)
                                    
    if config["delete"]:
        user_input = input("Are you sure about the deletion of the all processed original bag files? If it is copied, it will be also deleted. (yes/no): \n") # 
        if user_input.lower() == "yes":
            for file in processed_files:
                os.remove(file)
                print("Removed:", file)

        elif user_input.lower() == "no":
            print("Deletion is canceled.")

        else:
            print("Invalid input. Deletion is canceled.")

    

### TODO: progress bar
# def print_percentage_bar(iteration, total, prefix='', suffix='', decimals=1, length=100, fill='█'):
#     """
#     Call in a loop to create terminal progress bar
#     @params:
#         iteration   - Required  : current iteration (Int)
#         total       - Required  : total iterations (Int)
#         prefix      - Optional  : prefix string (Str)
#         suffix      - Optional  : suffix string (Str)
#         decimals    - Optional  : positive number of decimals in percent complete (Int)
#         length      - Optional  : character length of bar (Int)
#         fill        - Optional  : bar fill character (Str)
#     """
#     if total == 0:
#         total = 1
#     percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
#     filled_length = int(length * iteration // total)
#     bar = fill * filled_length + '-' * (length - filled_length)
#     print(f'\r{prefix} |{bar}| {percent}% {suffix}', end='\r')
#     # Print New Line on Complete
#     if iteration == total:
#         print()



if __name__ == "__main__":
    with open("config.yaml", "r") as f:
        config = yaml.safe_load(f)

    input_directory = config["input_directory"]
    output_directory = config["output_directory"]
    process_all_bag_files(input_directory, output_directory, config)
