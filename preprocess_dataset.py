import cv2
import time
import argparse
from threading import Thread
from os.path import join, exists
from os import listdir, getcwd
from utils import check_directory
from caffe_object_detection import caffe_get_detector, caffe_detect_body


def _tailor_image_pair(info_dict):
    col_min, col_max, row_min, row_max = info_dict['col_min'], info_dict['col_max'], info_dict['row_min'], info_dict['row_max']
    tailor_color_image(
        info_dict['old_color_filepath'],
        info_dict['new_color_filepath'],
        col_min, col_max, row_min, row_max
    )
    tailor_depth_image(
        info_dict['old_depth_filepath'],
        info_dict['new_depth_filepath'],
        col_min, col_max, row_min, row_max
    )
    print("Execute success:", info_dict['new_color_filepath'])
    # print("Active threads:", threading.active_count())


def preprocess_images(old_dataset_folder, new_dataset_folder):  
    execution_dir = getcwd()
    detector = caffe_get_detector(
        join(execution_dir, 'static', 'models', 'MobileNetSSD', 'MobileNetSSD_deploy.prototxt'),
        join(execution_dir, 'static', 'models', 'MobileNetSSD', 'MobileNetSSD_deploy.caffemodel')
    )
    old_color_folder = join(old_dataset_folder, 'color')
    old_depth_folder = join(old_dataset_folder, 'depth')
    new_color_folder = join(new_dataset_folder, 'color')
    new_depth_folder = join(new_dataset_folder, 'depth')
    check_directory(new_dataset_folder)
    check_directory(new_color_folder)
    check_directory(new_depth_folder)
    color_filenames = listdir(old_color_folder)
    depth_filenames = listdir(old_depth_folder)
    for color_filename, depth_filename in zip(color_filenames, depth_filenames):
        old_color_filepath = join(old_color_folder, color_filename)
        old_depth_filepath = join(old_depth_folder, depth_filename)
        new_color_filepath = join(new_color_folder, color_filename)
        new_depth_filepath = join(new_depth_folder, depth_filename)
        col_min, col_max, row_min, row_max = caffe_detect_body(
            detector=detector,
            image_path=old_color_filepath
        )
        print(f"Detect body: {old_color_filepath}")
        info_dict = {
            'col_min': col_min, 'col_max': col_max,
            'row_min': row_min, 'row_max': row_max,
            'old_color_filepath': old_color_filepath,
            'new_color_filepath': new_color_filepath,
            'old_depth_filepath': old_depth_filepath,
            'new_depth_filepath': new_depth_filepath,
        }
        thread = Thread(target=_tailor_image_pair, args=(info_dict, ))
        thread.start()
    thread.join()
    time.sleep(3)


def tailor_color_image(old_color_image_filename, new_color_image_filename,
                       col_min, col_max, row_min, row_max):
    img = cv2.imread(old_color_image_filename)
    rows, cols, _ = img.shape
    for col in range(cols):
        for row in range(rows):
            if not (col_min < col < col_max and row_min < row < row_max):
                img[row, col] = [0, 0, 0]
    cv2.imwrite(new_color_image_filename, img)


def tailor_depth_image(old_depth_image_filename, new_depth_image_filename,
                       col_min, col_max, row_min, row_max):
    img = cv2.imread(old_depth_image_filename, -1)
    rows, cols = img.shape
    for col in range(cols):
        for row in range(rows):
            if not (col_min < col < col_max and row_min < row < row_max):
                img[row, col] = 0
    cv2.imwrite(new_depth_image_filename, img)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--old-dataset", type=str, help="Dataset needed to be processed.")
    parser.add_argument("-n", "--new-dataset", type=str, help="Processed dataset's folder path.")
    args = parser.parse_args()
    if not exists(args.old_dataset):
        print("Dataset is not exist.")
        exit(-1)
    
    preprocess_images(
        old_dataset_folder=args.old_dataset,
        new_dataset_folder=args.new_dataset
    )