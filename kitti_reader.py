import os
import os.path as op
from glob import glob
import cv2

from reader_base import DriveManagerBase, DatasetReaderBase


class KittiDriveManager(DriveManagerBase):
    def __init__(self, datapath, split):
        super().__init__(datapath, split)

    def list_drive_paths(self):
        drive_paths = os.listdir(self.datapath)
        valid_paths = []
        self.check_validity(drive_paths)
        return valid_paths

    def check_validity(self, drive_path):
        demanded_drive_list = ["data_depth_annotated", "data_object_calib",
                               "data_object_image_2", "data_object_label_2", "data_object_velodyne"]
        drive_path.sort()
        demanded_drive_list.sort()
        diff_list = []
        if drive_path == demanded_drive_list:
            return True
        elif len(drive_path) > len(demanded_drive_list):
            for element in drive_path:
                if element not in demanded_drive_list:
                    diff_list.append(element)
            assert False, f"NEED {diff_list}"
        else:
            for element in demanded_drive_list:
                if element not in drive_path:
                    diff_list.append(element)
            assert False, f"DON'T NEED {diff_list}"

    def get_drive_name(self, drive_index):
        return f"drive{drive_index:02d}"


class KittiReader(DatasetReaderBase):
    def __init__(self, drive_path, split, dataset_cfg):
        super().__init__(drive_path, split, dataset_cfg)

    def init_drive(self, drive_path, split):
        frame_names = glob(op.join(drive_path, "*.png"))
        frame_names.sort()
        if split == "train":
            frame_names = frame_names[:-500]
        else:
            frame_names = frame_names[-500:]
        print("[KittiReader.init_drive] # frames:", len(frame_names),"first:", frame_names[0])
        return frame_names

    def get_images(self, index):
        return cv2.imread(self.frame_names[index])

    def get_box2d(self, index, sensor_id=None):
        image_file = self.frame_names[index]
        label_file = image_file.replace("image_2", "label_2").replace(".png", ".txt")


def test_kitti_reader():
    pass

kitti = KittiDriveManager('/media/cheetah/IntHDD/datasets/kitti', 'train')