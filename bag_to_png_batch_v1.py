import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os.path
import tkinter.filedialog
import os

def read_bag_filename(inputDir):
    dir_path = inputDir
    all_file = os.listdir(dir_path)
    all_bag_file = []
    for i in all_file:
        if os.path.splitext(i)[1] == '.bag':
            all_bag_file.append(i)
    return all_bag_file


def convert_bag_to_png(bagFile, dirPath, saveDir):
    bag = bagFile
    dir_path = dirPath
    pre_bag = os.path.splitext(bag)[0]
    input_path = os.path.join(dir_path, bag)
    # save_path = os.path.join(dir_path, '../bag_png',pre_bag)
    save_path = os.path.join(saveDir, pre_bag)
    screenshot_path = os.path.join(saveDir,'firstframe')
    print("save path:", save_path)
    if not os.path.isdir(screenshot_path):
        os.mkdir(screenshot_path)

    if not os.path.isdir(save_path):
        os.makedirs(save_path)

    if not os.path.isdir(os.path.join(save_path, 'color')):
        os.mkdir(os.path.join(save_path, 'color'))

    if not os.path.isdir(os.path.join(save_path, 'depth')):
        os.mkdir(os.path.join(save_path, 'depth'))

    if not os.path.isdir(os.path.join(save_path, 'colormap')):
        os.mkdir(os.path.join(save_path, 'colormap'))

    index = 0

    try:
        pipeline = rs.pipeline()
        config = rs.config()
        rs.config.enable_device_from_file(config, input_path, repeat_playback=False)

        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, rs.format.rgb8, 30)

        pipeline.start(config)

        align_to = rs.stream.color
        align = rs.align(align_to)

        colorizer = rs.colorizer

        while True:
            try:
                frames = pipeline.wait_for_frames()
            except RuntimeError as e:
                print('%s file convert finished, please check the png file output' % bag)
                print('each subdirectory has ', '%d' % index, 'png files (video 30fps)')
                break

            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            cv2.imwrite('%s/%s/%08d.png' % (save_path, 'color', index+1), color_image_rgb)
            cv2.imwrite('%s/%s/%08d.png' % (save_path, 'colormap', index+1), depth_colormap)
            cv2.imwrite('%s/%s/%08d.png' % (save_path, 'depth', index+1), depth_image)
            if index == 0:
                screenshot_image = np.hstack((color_image_rgb, depth_colormap))
                cv2.imwrite('%s/%s.png' % (screenshot_path, pre_bag), screenshot_image)

            index += 1
    finally:
        pass


if __name__ == '__main__':
    # input args
    parser = argparse.ArgumentParser(description='Convert bag file to png file')
    parser.add_argument('--input_dir', type=str, default=None, help='input bag file directory')
    parser.add_argument('--output_dir', type=str, default=None, help='output png file directory')
    args = parser.parse_args()

    input_dir = args.input_dir
    output_dir = args.output_dir
    print(input_dir)
    bag_file_list = read_bag_filename(input_dir)
    for bag_file in bag_file_list:
        convert_bag_to_png(bag_file, input_dir, output_dir)
    print('************ Finished ************')
