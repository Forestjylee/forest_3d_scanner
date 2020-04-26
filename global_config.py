import os
from configparser import ConfigParser


class BaseConfig(object):

    def __init__(self):
        self.config_file = 'config.ini'
        self.config_parser = ConfigParser()
        self.config_parser.read(os.path.join(os.getcwd(), self.config_file), encoding='utf-8')

        self.debug = False
        self.version = ''
        self.voxel_size = 0.0
        self.max_correspondence_distance_fine = 0.0
        self.max_correspondence_distance_coarse = 0.0

        self.is_output_point_cloud = False
        self.is_optimize_mesh = False
        self.optimization_iteration_count = 0

        self.camera_intrinsic_filename = ''
        self.pose_graph_filename = ''
        self.raw_mesh_filename = ''
        self.point_cloud_filename = ''
        self.optimized_mesh_filename = ''

    def _init_common_parameters(self):
        configs = self.config_parser[self.version]
        self.debug = False if configs['debug']=='no' else True
        self.voxel_size = float(configs['voxel_size'])
        self.max_correspondence_distance_coarse = float(configs['max_correspondence_distance_coarse_coefficient'])
        self.max_correspondence_distance_fine = float(configs['max_correspondence_distance_fine_coeficient'])
        self.is_output_point_cloud = False if configs['is_output_point_cloud']=='no' else True
        self.is_optimize_mesh = False if configs['is_optimize_mesh']=='no' else True
        self.optimization_iteration_count = int(configs['optimization_iteration_count'])

        self.pose_graph_filename = configs['pose_graph']
        self.camera_intrinsic_filename = configs['camera_intrinsic']
        self.raw_mesh_filename = configs['raw_mesh_filename']
        self.point_cloud_filename = configs['point_cloud_filename']
        self.optimized_mesh_filename = configs['optimized_mesh_filename']



class OnlineConfig(BaseConfig):

    def __init__(self):
        super(OnlineConfig, self).__init__();
        self.version = 'online'
        self.only_body = False
        self.images_count = 0
        self.max_depth_in_meters = 0.0
        self.icp_type = ''
        self.output_folder = ''

        self._init_common_parameters()
        self._init_particular_parameters()

    def _init_particular_parameters(self):
        configs = self.config_parser[self.version]
        self.images_count = int(configs['images_count'])
        if configs['icp_type'] not in ['point_to_plane', 'color']:
            raise ValueError("icp_type in config.ini is not support, please choose another one.")
        self.icp_type = configs['icp_type']
        self.max_depth_in_meters = float(configs['max_depth_in_meters'])

        if not os.path.isabs(configs['output_folder']):
            self.output_folder = os.path.join(os.getcwd(), configs['output_folder'])
        else:
            self.output_folder = configs['output_folder']


class OfflineConfig(BaseConfig):
    
    def __init__(self):
        super(OfflineConfig, self).__init__()
        self.version = 'offline'
        self.dataset = ''

        self._init_common_parameters()
        self._init_particular_parameters()

    def _init_particular_parameters(self):
        configs = self.config_parser[self.version]
        
        if not os.path.isabs(configs['dataset']):
            self.dataset = os.path.join(os.getcwd(), configs['dataset'])
        else:
            self.dataset = configs['dataset']
