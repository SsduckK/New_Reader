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
        return op.join(self.datapath, "data_object_image_2", self.split)

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
        self.image_file = None
        self.label_file = None
        self.velo_file = None
        self.cali_file = None

    def init_drive(self, drive_path, split):
        frame_names = glob(op.join(drive_path, "*.png"))
        frame_names.sort()
        if split == "train":
            frame_names = frame_names[:-500]
        else:
            frame_names = frame_names[-500:]
        print("[KittiReader.init_drive] # frames:", len(frame_names), "first:", frame_names[0])
        return frame_names

    def get_images(self, index):
        return cv2.imread(self.frame_names[index])

    def get_box2d(self, index, raw_image_shape=None, sensor_id=None):
        self.image_file = self.frame_names[index]
        self.label_file = self.image_file.replace("image_2", "label_2").replace(".png", ".txt")
        self.cali_file = self.image_file.replace("image_2", "calib").replace(".png", ".txt")
        instrinsic,  calib_velo_to_cam= self.get_intrinsic()
        point_cloud = self.get_point_cloud(index=index)
        depth_map = self.get_depth_map()

    def get_intrinsic(self, sensor_id=None):
        calib_data = self.load_calib_data(self.cali_file)
        instrinsic = calib_data["P0"].copy()[:, :3]
        return instrinsic, calib_data["Tr_velo_to_cam"]

    def load_calib_data(self, file):
        calib_dict = dict()
        with open(file, "r") as f:
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
        calib_dict["Tr_velo_to_cam"] = np.reshape(np.array(calib_dict["Tr_velo_to_cam"]), (3, 4))
        calib_dict["P0"] = np.reshape(np.array(calib_dict["P0"]), (3, 4))
        return calib_dict

    def load_velo_scan(self, file):
        scan = np.fromfile(file, dtype=np.float32)
        scan = scan.reshape((-1, 4))
        return scan

    def get_point_cloud(self, index, sensor_id=None, frame=None, style=None):
        velo_file = self.image_file.replace("image_2", "velodyne").replace(".png", ".bin")

        velo_data = self.load_velo_scan(velo_file)

    def get_depth_map(self, index, sensor_id=None, frame=None):
        pass

def test_kitti_reader():
    print("===== start test_kitti_reader")
    srcpath = "/media/cheetah/IntHDD/datasets/kitti"
    drive_mngr = KittiDriveManager(srcpath, "train")
    drive_paths = drive_mngr.get_drive_paths()
    reader = KittiReader(drive_paths, 'train', 'temp')
    for i in range(reader.__len__()):
        image = reader.get_images(i)
        bboxes_2d = reader.get_box2d(i)


test_kitti_reader()
