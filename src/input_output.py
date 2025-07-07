import numpy as np
import scipy.spatial.transform as transf
import os
import random
import utils

import geometry
import Simulator

def read_poses(file_name:str):
    data = np.loadtxt(file_name, dtype = str, skiprows = 1, delimiter = ',')
    assert data.shape[1] == 8, 'Error! File with poses is wrong or missformated. Data in the file must be interpretable as array with 7 columns.'
    poses = {}
    for pose_entry in data:
        id = int(pose_entry[0])
        position = np.array(pose_entry[1:4].astype(float)).reshape(3,1)
        rotation = transf.Rotation.from_euler('ZYX',pose_entry[4:7].astype(float),True)
        type_of_pose = pose_entry[7]
        poses[id] = geometry.Pose(position, rotation, type_of_pose)
    return poses

def read_graph(file_name:str):
    data = np.loadtxt(file_name, dtype = int, skiprows = 1, delimiter = ',')
    assert data.shape[1] == 2, 'Error! File with graph edges is wrong or missformated. Data in the file must be interpretable as array with 2 columns.'
    edges = []
    nodes = set()
    for edge_data in data:
        nodes.add(edge_data[0])
        nodes.add(edge_data[1])
        edges.append(geometry.GraphEdge(edge_data[0], edge_data[1]))
    graph = geometry.SimpleVisibilityGraph(nodes = nodes)
    for edge in edges:
        assert graph.try_add_edge(edge), 'error while constructing visibilit graph'
    return graph

def print_features(simulator:Simulator):
    for (pose_id, features) in simulator.dict_of_features.items():
        print(f'{pose_id=}')
        for feature in features:
            print(f'{feature.id=} : {feature.position[0,0]:.3f} {feature.position[1,0]:.3f} {feature.position[2,0]:.3f}  {feature.visibility=}')
            
def save_features(output_direcotry:str, simulator:Simulator):
    path_to_output_file = os.path.join(output_direcotry, 'lidar_measurements.txt')
    with open(path_to_output_file,'w') as file:
        header = 'pose_id,feature_id,x,y,z,sigma_x,sigma_y,sigma_z\n'
        uncertainty = simulator.config.gaussian_noise_point_position
        file.write(header)
        for (pose_id, features) in simulator.dict_of_features.items():
            for feature in features:
                file.write('%d,' % pose_id)
                file.write('%d,' % feature.id)
                file.write('%.4f,%.4f,%.4f,' % (feature.position[0,0], feature.position[1,0], feature.position[2,0]) )
                file.write('%.4f,%.4f,%.4f\n' % (uncertainty, uncertainty, uncertainty) )
                
                
def save_all_feature_data(output_direcotry:str, simulator:Simulator, poses:dict[int,geometry.Pose]):
    path_to_output_file = os.path.join(output_direcotry, 'feature_data.txt')
    with open(path_to_output_file,'w') as file:
        header = 'pose_id,feature_id,number_of_measurements,x,y,z,x_global,y_global,z_global\n'
        file.write(header)
        for (pose_id, features) in simulator.dict_of_features.items():
            for feature in features:
                pose = poses[pose_id]
                global_coordinates = pose.T()@feature.as_homogenous_vector()
                file.write('%d,' % pose_id)
                file.write('%d,' % feature.id)
                file.write('%d,' % len(feature.visibility))
                file.write('%.4f,%.4f,%.4f,' % (feature.position[0,0], feature.position[1,0], feature.position[2,0]) )
                file.write('%.4f,%.4f,%.4f\n' % (global_coordinates[0,0], global_coordinates[1,0], global_coordinates[2,0]))


def save_poses(output_direcotry:str, poses:dict[int,geometry.Pose], position_rotation_noise ):
    path_to_output_file = os.path.join(output_direcotry, 'poses.txt')
    deg_to_rad = np.pi/180.0
    sigma_pos = position_rotation_noise[0]
    sigma_rot = position_rotation_noise[1]*deg_to_rad
    random.seed()
    with open(path_to_output_file,'w') as file:
        header = 'id,x,y,z,qw,qx,qy,qz,type\n'
        file.write(header)
        for pose_id, pose in poses.items():
            noise_position = np.zeros((3,1))
            noise_rotation = np.eye(3)
            if sigma_pos > 0.0 and pose.type == 'free':
                vx = utils.gaussian_noise_with_limit(sigma_pos)
                vy = utils.gaussian_noise_with_limit(sigma_pos)
                vz = utils.gaussian_noise_with_limit(sigma_pos)
                noise_position += np.array([vx, vy, vz]).reshape((3,1))
            if sigma_rot > 0.0 and pose.type == 'free':
                noise_rotation = utils.generate_noise_on_so3(sigma_rot)
            position = pose.position + noise_position
            rotation = transf.Rotation.from_matrix(pose.rotation.as_matrix()@noise_rotation)          
            pose_with_noise = geometry.Pose(position, rotation)
            quat = pose_with_noise.get_rotation_as_wxyz_quaternion()
            file.write('%d,' % pose_id)
            file.write('%.4f,%.4f,%.4f,' % (pose_with_noise.position[0,0], pose_with_noise.position[1,0], pose_with_noise.position[2,0]))
            file.write('%.15f,%.15f,%.15f,%.15f,' % (quat[0], quat[1], quat[2], quat[3]))
            file.write('%s\n' % pose.type)
 
def save_network_to_dxf(output_direcotry:str, simulator:Simulator, poses:dict[int,geometry.Pose]):
    path_to_output_file = os.path.join(output_direcotry, 'network.dxf')
    with open(path_to_output_file,'w') as _:
        pass
    initialize_dxf_entities(path_to_output_file)
    save_poses_to_dxf(path_to_output_file, simulator, poses)
    save_rays_to_dxf(path_to_output_file, simulator, poses)
    close_dxf_entities(path_to_output_file)
    
def initialize_dxf_entities(dxf_filename:str):
    with open(dxf_filename,'a') as file:
        file.write('0\n')
        file.write('SECTION\n')
        file.write('2\n')
        file.write('HEADER\n')
        file.write('9\n')
        file.write('$ACADVER\n')
        file.write('1\n')
        file.write('AC1009\n')
        file.write('0\n')
        file.write('ENDSEC\n')
        file.write('0\n')
        file.write('SECTION\n')
        file.write('2\n')
        file.write('ENTITIES\n')
        file.write('0\n')
        
def close_dxf_entities(dxf_filename:str):
    with open(dxf_filename,'a') as file:
        file.write('ENDSEC\n')
        file.write('0\n') 
                
def save_poses_to_dxf(dxf_filename:str, simulator:Simulator, poses:dict[int,geometry.Pose]):  
    with open(dxf_filename,'a') as file:
        colors_of_the_axes = ['1','3','5']
        for _,pose in poses.items():
            axes = simulator.config.axes_length_in_dxf * pose.rotation.as_matrix()
            for i in range(0,3):
                file.write('LINE\n')
                file.write('8\n')
                file.write('cs\n')
                file.write('39\n')
                file.write('4\n')
                file.write('62\n')
                file.write('%s\n' % colors_of_the_axes[i])
                file.write('10\n')
                file.write('%.5f\n' % (pose.position[0,0]))
                file.write('20\n')
                file.write('%.5f\n' % (pose.position[1,0]))
                file.write('30\n')
                file.write('%.5f\n' % (pose.position[2,0]))
                file.write('11\n')
                file.write('%.5f\n' % (pose.position[0,0] + axes[0,i]))
                file.write('21\n')
                file.write('%.5f\n' % (pose.position[1,0] + axes[1,i]))
                file.write('31\n')
                file.write('%.5f\n' % (pose.position[2,0] + axes[2,i]))
                file.write('0\n')
            
def save_rays_to_dxf(dxf_filename:str, simulator:Simulator, poses:dict[int,geometry.Pose]):
    with open(dxf_filename,'a') as file:
        for pose_id, features in simulator.dict_of_features.items():
            pose = poses[pose_id]
            for feature in features:
                feature_in_world = pose.T()@feature.as_homogenous_vector()
                file.write('LINE\n')
                file.write('8\n')
                file.write('rays\n')
                file.write('39\n')
                file.write('4\n')
                file.write('62\n')
                file.write('8\n')
                file.write('10\n')
                file.write('%.5f\n' % (pose.position[0,0]))
                file.write('20\n')
                file.write('%.5f\n' % (pose.position[1,0]))
                file.write('30\n')
                file.write('%.5f\n' % (pose.position[2,0]))
                file.write('11\n')
                file.write('%.5f\n' % (feature_in_world[0,0]))
                file.write('21\n')
                file.write('%.5f\n' % (feature_in_world[1,0]))
                file.write('31\n')
                file.write('%.5f\n' % (feature_in_world[2,0]))
                file.write('0\n')