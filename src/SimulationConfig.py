import xml.etree.ElementTree as ET

class Config:
    def __init__(self):
        self.number_of_features_per_cloud = 2000
        self.gaussian_noise_point_position = 0.01
        self.gaussian_noise_angle_deg = 0.01
        self.gaussian_noise_distance = 0.003
        self.use_anisotropic_noise = True
        self.matching_probability = 0.2
        self.min_distance = 2.5
        self.max_distance = 3.1
        self.min_vertical_angle_deg = -30
        self.max_vertical_angle_deg = 90
        self.pose_noise_position = 0.05
        self.pose_noise_rotation_deg = 3.0
        self.axes_length_in_dxf = 1.0
       
    def read_from_xml(self,path_to_xml_file):
        tree = ET.parse(path_to_xml_file)
        config = tree.getroot()
        self.number_of_features_per_cloud = int(config.find('number_of_features_per_cloud').text)
        self.gaussian_noise_point_position = float(config.find('gaussian_noise_point_position').text)
        self.gaussian_noise_angle_deg = float(config.find('gaussian_noise_angle_deg').text )
        self.gaussian_noise_distance = float(config.find('gaussian_noise_distance').text )
        self.use_anisotropic_noise = config.find('use_anisotropic_noise').text == 'True'
        self.matching_probability = float(config.find('matching_probability').text)
        self.min_distance = float(config.find('min_distance').text)
        self.max_distance = float(config.find('max_distance').text)
        self.min_vertical_angle_deg = float(config.find('min_vertical_angle_deg').text)
        self.max_vertical_angle_deg = float(config.find('max_vertical_angle_deg').text)
        self.pose_noise_position = float(config.find('pose_noise_position').text)
        self.pose_noise_rotation_deg = float(config.find('pose_noise_rotation_deg').text)
        self.axes_length_in_dxf = float(config.find('axes_length_in_dxf').text)
        assert self.number_of_features_per_cloud > 0
        assert self.gaussian_noise_point_position >= 0.0
        assert self.gaussian_noise_angle_deg >= 0.0
        assert self.gaussian_noise_distance >= 0.0
        assert self.matching_probability > 0
        assert self.matching_probability < 1
        assert self.max_distance > 0
        assert self.min_distance < self.max_distance
        assert self.min_vertical_angle_deg >= 0
        assert self.max_vertical_angle_deg <= 180
        assert self.min_vertical_angle_deg < self.max_vertical_angle_deg
        assert self.pose_noise_position >= 0.0
        assert self.pose_noise_rotation_deg >= 0.0
        assert self.axes_length_in_dxf > 0.0
        
    number_of_features_per_cloud = 2000
    gaussian_noise_point_position = 0.01
    gaussian_noise_angle_deg = 0.01
    gaussian_noise_distance = 0.003
    matching_probability = 0.2
    min_distance = 2.5
    max_distance = 3.1
    min_vertical_angle_deg = -30
    max_vertical_angle_deg = 90
    