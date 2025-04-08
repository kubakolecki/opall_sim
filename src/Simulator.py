import utils
import SimulationConfig
import geometry

import random
import numpy as np


class Simulator(object):
    def __init__(self, config:SimulationConfig):
        self.config = config
        self.point_id = 1
        self.dict_of_features = {}
        self.dict_of_poses_visibility = {}
        self.dict_of_feature_counts = {}
        
    def repeat_range(self, n:int):
        while True:
            for i in range(n):
                yield i
        
    def run_simulations(self, poses:dict[int,geometry.Pose], graph:geometry.SimpleVisibilityGraph):
        self.point_id = 1
        self.dict_of_poses_visibility = self.create_dict_of_poses_visibility(graph)
        self.generate_features(poses)
        self.add_noise_to_feature_coordinates()
                
    def generate_random_feature(self, feature_id:int):
        horizontal_angle = random.uniform(0, 2*np.pi)
        vertical_angle = utils.deg_to_rad(random.uniform(self.config.min_vertical_angle_deg, self.config.max_vertical_angle_deg))
        slant_distance = random.uniform(self.config.min_distance, self.config.max_distance)
        horizontal_distance = slant_distance*np.cos(slant_distance)
        x = horizontal_distance*np.cos(horizontal_angle)
        y = horizontal_distance*np.sin(horizontal_angle)
        z = slant_distance*np.sin(vertical_angle)
        feature = geometry.FeatureIn3d(id = feature_id, position = np.array([x,y,z]).reshape((3,1)), uncertainty = self.config.gaussian_noise)
        return feature
    
    def generate_features(self, poses:dict[int,geometry.Pose]):
        random.seed()
        number_of_poses = len(poses)
        pose_ids = list(poses.keys())
        total_number_of_features = self.config.number_of_features_per_cloud * number_of_poses      
        pose_id_generator = self.repeat_range(number_of_poses)
        
        #populating dictionary of features with empty lists:
        self.dict_of_features = {} #clearing
        for pose_id in pose_ids:
            self.dict_of_features[pose_id] = []

        for feature_id in range(1,total_number_of_features+1):
            pose_index = next(pose_id_generator)
            reference_pose_id = pose_ids[pose_index] #id of current pose
            reference_pose = poses[reference_pose_id]
            feature_visible_from_reference_pose = self.generate_random_feature(feature_id)
            set_of_poses_feature_is_visible_from = set()
            set_of_poses_feature_is_visible_from.add(reference_pose_id)
            self.dict_of_features[reference_pose_id].append(feature_visible_from_reference_pose)
            ids_of_visible_poses = self.dict_of_poses_visibility[reference_pose_id]
            for query_pose_id in ids_of_visible_poses:
                rnd = random.uniform(0.0,1.0) 
                if rnd > self.config.matching_probability:
                    continue #sorry, this feature is not visible, skipp
                visibilit_conflict = False #Features can not be co-visible if poses are not co-visible
                for id in set_of_poses_feature_is_visible_from:
                    if not (id in self.dict_of_poses_visibility[query_pose_id]):
                        visibilit_conflict = True
                if visibilit_conflict:
                    continue
                query_pose = poses[query_pose_id]
                point_in_query = (query_pose.T_inv()@reference_pose.T())@feature_visible_from_reference_pose.as_homogenous_vector()
                feature_visible_from_query_pose = geometry.FeatureIn3d(id = feature_id, position = point_in_query[0:3,:], uncertainty = self.config.gaussian_noise) 
                feature_in_world_q = query_pose.T()@feature_visible_from_query_pose.as_homogenous_vector()
                feature_in_world_r = reference_pose.T()@feature_visible_from_reference_pose.as_homogenous_vector()
                assert np.allclose(feature_in_world_q, feature_in_world_r, rtol=1e-05, atol=1e-08, equal_nan=False), "check for world coordinate consistency failed!" 
                set_of_poses_feature_is_visible_from.add(query_pose_id)
                self.dict_of_features[query_pose_id].append(feature_visible_from_query_pose)
            for pose_id in set_of_poses_feature_is_visible_from:
                self.dict_of_features[pose_id][-1].visibility = set_of_poses_feature_is_visible_from
            if len(set_of_poses_feature_is_visible_from) == 1: #this feature was not matched, removing it
                self.dict_of_features[reference_pose_id].pop()
                    
    def create_dict_of_poses_visibility(self, graph:geometry.SimpleVisibilityGraph):
        dict_of_poses_visibility = {}
        for node in graph.nodes:
           visible_nodes = set()
           for edge in graph.edges:
               if node == edge.from_id:
                   visible_nodes.add(edge.to_id)
               if node == edge.to_id:
                   visible_nodes.add(edge.from_id)
           dict_of_poses_visibility[node] = visible_nodes
        return dict_of_poses_visibility

    def add_noise_to_feature_coordinates(self):
        if self.config.gaussian_noise == 0.0:
            return
        random.seed()
        for (_, features) in self.dict_of_features.items():
            for feature in features:
                feature.position[0,0] += utils.gaussian_noise_with_limit(self.config.gaussian_noise)
                feature.position[1,0] += utils.gaussian_noise_with_limit(self.config.gaussian_noise)
                feature.position[2,0] += utils.gaussian_noise_with_limit(self.config.gaussian_noise)



