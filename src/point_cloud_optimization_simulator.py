import sys
import os
import getopt

import SimulationConfig
import Simulator
import input_output as io

def main():
    path_file_config_xml = ''
    path_file_poses = ''
    path_file_graph = ''
    path_directory_output = ''
    try:
      opts, args = getopt.getopt(sys.argv[1:],'c:p:g:o:',['config=','poses=', 'graph=', 'output='])
    except:
      print ('error while parsing command line arguments')
      print ('you need to provid all 4 arguments:')
      print ('-c, --config : path to xml config file')
      print ('-p, --poses : path to textfile with poses')
      print ('-g, --graph : path to textfile with graph edges')
      print ('-o, --output : path to output directory, where the results will be saved')
      sys.exit(2)
    for opt, arg in opts:
      if opt in ('-c', '--config'):
         path_file_config_xml = arg
      elif opt in ('-p', '--poses'):
         path_file_poses = arg
      elif opt in ('-g', '--graph'):
         path_file_graph = arg
      elif opt in ('-o', '--output'):
         path_directory_output = arg
      else:
         assert False, 'unhandled option'
         
    if not os.path.exists(path_file_config_xml):
       print('Config xml file does not exist!')
       sys.exit(2)
       
    if not os.path.exists(path_file_poses):
       print('Poses file does not exist!')
       sys.exit(2)
    
    if not os.path.exists(path_file_graph):
       print('Graph file does not exist!')
       sys.exit(2)
       
    if not os.path.exists(path_directory_output):
       print('Output directory does not exist!')
       sys.exit(2)
       
    config = SimulationConfig.Config()    
    config.read_from_xml(path_file_config_xml)
    
    poses = io.read_poses(path_file_poses)
    visibility_graph = io.read_graph(path_file_graph)
    
    #checking if all poses are represented by nodes in the visibilit graph
    set_of_pose_ids = set()
    for pose_id in poses.keys():
       set_of_pose_ids.add(pose_id)
    assert set_of_pose_ids == visibility_graph.nodes, 'Error while checking visibility graph! ids of poses must match nodes in visibility graph!'
    
    print(f'Created visibility graph with {len(visibility_graph.nodes)} nodes and {len(visibility_graph.edges)} edges.')

    uconnected_nodes = visibility_graph.verify_node_connections()

    if len(uconnected_nodes) != 0:
       print('Visibility graph is invalid!')
       print(f'{len(uconnected_nodes)} nodes of the graph are not connected.')
       print('Exiting.')
       sys.exit(2)
       
    simulator = Simulator.Simulator(config)
    simulator.run_simulations(poses, visibility_graph)
    io.print_features(simulator)
    io.save_features(path_directory_output, simulator)
    io.save_all_feature_data(path_directory_output, simulator, poses)
    io.save_network_to_dxf(path_directory_output, simulator, poses)
    io.save_poses(path_directory_output, poses, (config.pose_noise_position, config.pose_noise_rotation_deg))

if __name__ == '__main__':
   main()
