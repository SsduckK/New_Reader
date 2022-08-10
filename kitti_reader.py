import os
import os.path as op
from glob import glob
import cv2
import numpy as np

from reader_base import DriveManagerBase, DatasetReaderBase


class KittiDriveManager(DriveManagerBase):
    def __init__(self, datapath, split):
        super().__init__(datapath, split)

    def list_drive_paths(self):
        drive_paths = os.listdir(self.datapath)
        valid_paths = []
        if self.check_validity(drive_paths):
            valid_paths = drive_paths
        return [op.join(self.datapath, drive, self.split) for drive in valid_paths]

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
        self.cali_file = glob(op.join(drive_path[1], "*.txt"))
        self.label_file = glob(op.join(drive_path[3], "*.txt"))
        self.velo_file = glob(op.join(drive_path[4], "*.bin"))
        self.calib_data = None
        self.file_index = None

    def init_drive(self, drive_path, split):
        """
        :param drive_path:
        datapath +
        ["data_depth_annotated", "data_object_calib",
        "data_object_image_2", "data_object_label_2", "data_object_velodyne"]
        + split
        :param split: train/test
        :return: [self.image_file]
        """
        self.image_file = glob(op.join(drive_path[2], "*.png"))

        self.image_file.sort()
        if split == "train":
            self.image_file = self.image_file[:-500]
        else:
            self.image_file = self.image_file[-500:]
        print("[KittiReader.init_drive] # frames:", len(self.image_file), "first:", self.image_file[0])
        return self.image_file

    def __len__(self):
        """
        :return: numbers of frame_names
        """
        return len(self.frame_names)

    def get_images(self, index, sensor_id=None):
        """
        :param index: lists of file's index
        :param sensor_id: using when there is a multiple sensor
        :return: numpy image of self.frame_names[index]
        """
        return cv2.imread(self.frame_names[index])

    def get_box2d(self, index, sensor_id=None):
        """
        :param index: lists of file's index
        :param sensor_id: using when there is a multiple sensor
        :return:
        """
        image_file = self.frame_names[index]
        label_file = self.label_file.copy()[index]
        bboxes = []
        categories = []

        with open(label_file, 'r') as f:
            lines = f.readlines()
            for i, line in enumerate(lines):
                bbox, category = self.extract_box(line)
                if bbox is not None:
                    bboxes.append(bbox)
                    categories.append(category)
        bboxes = np.array(bboxes)
        return bboxes, categories

    def extract_box(self, line):
        raw_label = line.strip("\n").split(" ")
        category_name = raw_label[0]
        if category_name not in self.dataset_cfg.CATEGORIES_TO_USE:
            return None, None
        category_index = self.dataset_cfg.CATEGORIES_TO_USE.index(category_name)
        if category_name in self.dataset_cfg.CATEGORY_REMAP:
            category_name = self.dataset_cfg.CATEGORY_REMAP[category_name]
        y1 = round(float(raw_label[5]))
        x1 = round(float(raw_label[4]))
        y2 = round(float(raw_label[7]))
        x2 = round(float(raw_label[6]))
        depth = depth_map[y1:y2, x1:x2]
        dist_value = depth[np.where(depth > 0)]
        if dist_value.size == 0:
            dist_value = 0
        else:
            dist_value = np.quantile(dist_value, self.dataset_cfg.DIST_QUANTILE)
        bbox = np.array([(y1 + y2) / 2, (x1 + x2) / 2, y2 - y1, x2 - x1, 1, dist_value], dtype=np.int32)
        return bbox, category_name

    def get_box3d(self, index, sensor_id=None, frame=None, style=None):
        image_file = self.frame_names[index]

    def get_point_cloud(self, index, sensor_id=None, frame=None, style=None):
        self.calib_data = self.load_calib_data(index, self.cali_file)
        velo_data = self.load_velo_scan(index, self.velo_file)

        velo_data[:, 3] = 1
        velo_in_camera = np.dot(self.calib_data["Tr_velo_to_cam"], velo_data.T)
        velo_in_camera = velo_in_camera[:3].T
        velo_in_camera = velo_in_camera[velo_in_camera[:, 2] > 0]
        return velo_in_camera

    def load_calib_data(self, index, file):
        """
        loading calibration data file .txt
        :param index: lists of file's index
        :param file: calibration data file
        :return: calibration matrix with dictionary ndarray (3 x n)
        """
        calib_dict = {}
        with open(file[index], "r") as f:
            lines = f.readlines()
            for line in lines:
                new_line = []
                line = line.split(" ")
                if len(line) == 1:
                    pass
                else:
                    line[0] = line[0].rstrip(":")
                    line[-1] = line[-1].rstrip("\n")
                    for a in line[1:]:
                        new_line.append(float(a))
                    calib_dict[line[0]] = new_line
        for key in list(calib_dict.keys()):
            calib_dict[key] = np.reshape(np.array(calib_dict[key]), (3, -1))
        return calib_dict

    def load_velo_scan(self, index, file):
        """Load and parse a velodyne binary file."""
        scan = np.fromfile(file[index], dtype=np.float32)
        scan = scan.reshape((-1, 4))
        return scan

    def get_intrinsic(self, index, sensor_id=None):
        """
        :param index: lists of file's index
        :param sensor_id: using when there is a multiple sensor
        :return: intrinsic matrix (3 x 4)
        """
        intrinsic = self.calib_data["P0"].copy()[:, :3]
        return intrinsic


def test_kitti_reader():
    print("===== start test_kitti_reader")
    srcpath = "/media/cheetah/IntHDD/datasets/kitti"
    drive_mngr = KittiDriveManager(srcpath, "train")
    drive_paths = drive_mngr.get_drive_paths()
    reader = KittiReader(drive_paths, 'train', 'temp')
    for i in range(reader.__len__()):
        image = reader.get_images(i)
        bboxes_2d = reader.get_box2d(i)
        reader.get_point_cloud(i)
        reader.get_intrinsic(i)


test_kitti_reader()
