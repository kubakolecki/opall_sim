import numpy as np
import scipy.spatial.transform as transf
import copy as cp

def spherical_to_cartesian(slant_distance,horizontal_angle_rad, vertical_angle_rad ):
    horizontal_distance = slant_distance*np.sin(vertical_angle_rad)
    x = horizontal_distance*np.cos(horizontal_angle_rad)
    y = horizontal_distance*np.sin(horizontal_angle_rad)
    z = slant_distance*np.cos(vertical_angle_rad)
    return x, y, z

def cartesian_to_spherical(x, y, z):
    slant_distance = np.sqrt(x*x + y*y + z*z)
    horizontal_angle_rad = np.atan2(y, x)
    vertical_angle_rad = np.acos(z/slant_distance)
    return slant_distance, horizontal_angle_rad, vertical_angle_rad

def get_jacobian_of_mapping_to_cartesian_coordinates(slant_distance, horizontal_angle_rad, vertical_angle_rad):
    ch = np.cos(horizontal_angle_rad)
    sh = np.sin(horizontal_angle_rad)
    cv = np.cos(vertical_angle_rad)
    sv = np.sin(vertical_angle_rad)
    jacobian = np.array([[sv*ch, -slant_distance*sv*sh, slant_distance*cv*ch],
                         [sv*sh,  slant_distance*sv*ch, slant_distance*cv*sh],
                         [cv, 0.0, -slant_distance*sv]])
    return jacobian



class Pose:
    def __init__(self, position = np.zeros((3,1)), rotation = transf.Rotation([0,0,0,1]), type:str = 'free' ):
        self.position = position
        self.rotation = rotation
        self.type = type #free, fixed
                
    def set_position(self,position):
        self.position = position
        
    def set_rotation_from_euler(self, yaw_pitch_roll):
        self.rotation = transf.Rotation.from_euler('ZYX', yaw_pitch_roll, False)
        
    def get_rotation_as_wxyz_quaternion(self):
        xyzw_quaternion = self.rotation.as_quat()
        wxyz_quaternion = [xyzw_quaternion[3], xyzw_quaternion[0], xyzw_quaternion[1], xyzw_quaternion[2]]
        return wxyz_quaternion
    
    def T(self): #converts pose to transformation matrix
        T = np.block([[self.rotation.as_matrix(), self.position],[np.zeros((1,3)), np.eye(1)]])
        return T
    
    def T_inv(self): #converts pose to the inverse transfomration matrix
        rt = np.transpose(self.rotation.as_matrix())
        T_inv = np.block([[rt, -rt@self.position],[np.zeros((1,3)), np.eye(1)]])
        return T_inv
    
class FeatureIn3d:
    def __init__(self,*, id:int = 0, position = np.zeros((3,1)), uncertainty:float = 0.01, covariance:np.array  ):
        self.id = id
        self.position = position
        self.uncertainty = uncertainty
        self.covariance = covariance #this is 'full', 3 x 3 covariance matrix
        self.visibility = set() #which stations this feature is visible from
        
    def as_homogenous_vector(self): #returns position as 4 x 1 homogenous vector
        homogenous_vector = np.block([[self.position],[1]])
        return homogenous_vector
    
class GraphEdge:
    def __init__(self, from_id:int, to_id:int):
        self.from_id = from_id
        self.to_id = to_id

class SimpleVisibilityGraph:
    def __init__(self, nodes = set()):
        self.nodes = nodes
        self.edges = []
        
    def try_add_edge(self, edge_to_add:GraphEdge):
        if not edge_to_add.from_id in self.nodes:
            return False
        if not edge_to_add.to_id in self.nodes:
            return False
        for existing_edge in self.edges:
            if (existing_edge.from_id == edge_to_add.from_id and existing_edge.to_id == edge_to_add.to_id):
                return False
            if (existing_edge.from_id == edge_to_add.to_id and existing_edge.to_id == edge_to_add.from_id):
                return False
        self.edges.append(edge_to_add)
        return True
    
    def verify_node_connections(self):
        connected_nodes = set()
        connected_nodes.add(list(self.nodes)[0])
        found_connected_node = True
        edges_to_check = cp.deepcopy(self.edges)
 
        while found_connected_node:
            found_connected_node = False
            for edge in edges_to_check:
                if (edge.from_id in connected_nodes) and not (edge.to_id in connected_nodes):
                    connected_nodes.add(edge.to_id)
                    found_connected_node = True
                    edges_to_check.remove(edge)
                    break
                if (edge.to_id in connected_nodes) and not (edge.from_id in connected_nodes):
                    connected_nodes.add(edge.from_id)
                    found_connected_node = True
                    edges_to_check.remove(edge)
                    break
        
        unconnected_nodes = self.nodes.difference(connected_nodes) 

        return unconnected_nodes
        
    
    nodes = set()
    edges = list()