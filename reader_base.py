import os
import os.path as op
from glob import glob


class DriveManagerBase:
    def __init__(self, datapath, split):
        self.datapath = datapath
        self.split = split
        self.drive_paths = self.list_drive_paths()

    def list_drive_paths(self):
        raise NotImplementedError()

    def check_validity(self, drive_path):
        raise NotImplementedError()

    def get_drive_paths(self):
        return self.drive_paths

    def get_drive_name(self, drive_index):
        raise NotImplementedError()


class DatasetReaderBase:
    def __init__(self, drive_path, split, dataset_cfg):
        self.dataset_cfg = dataset_cfg
        self.frame_names = self.init_drive(drive_path, split)

    def init_drive(self, drive_path, split):
        raise NotImplementedError()

    def __len__(self):
        return len(self.frame_names)

    def get_images(self, index):
        raise NotImplementedError()

    def get_box2d(self, index, raw_image_shape=None, sensor_id=None):
        pass

    def get_box3d(self, index, sensor_id=None, frame=None, style=None):
        pass

    def get_point_cloud(self, index, sensor_id=None, frame=None, style=None):
        pass

    def get_depth_map(self, index, sensor_id=None, frame=None):
        pass

    def get_intrinsic(self, sensor_id=None):
        pass

    def get_transform(self, src_frame, dst_frame, style=None):
        pass

    def get_transform_to_body(self, tgt_frame, style=None):
        pass

