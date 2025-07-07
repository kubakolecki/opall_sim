import geometry as geo
import numpy as np

nodes_ids = {1,2,3,4,5,6}
nodes_ids_2 = {1,2,3,4,5,6,1}

assert nodes_ids == nodes_ids_2 

print("testing graph")
graph_banana = geo.SimpleVisibilityGraph(nodes_ids)
assert graph_banana.try_add_edge(geo.GraphEdge(1,2)) == True
assert graph_banana.try_add_edge(geo.GraphEdge(2,3)) == True
assert graph_banana.try_add_edge(geo.GraphEdge(3,4)) == True
assert graph_banana.try_add_edge(geo.GraphEdge(4,6)) == True
assert graph_banana.try_add_edge(geo.GraphEdge(3,5)) == True
assert graph_banana.try_add_edge(geo.GraphEdge(5,2)) == True
assert graph_banana.try_add_edge(geo.GraphEdge(1,2)) == False #this one already exists
assert graph_banana.try_add_edge(geo.GraphEdge(3,2)) == False #the inverse of this one already exists
assert graph_banana.try_add_edge(geo.GraphEdge(6,7)) == False #node with id 7 does not exist
assert graph_banana.try_add_edge(geo.GraphEdge(8,1)) == False #node with id 8 does not exist
assert graph_banana.try_add_edge(geo.GraphEdge(8,0)) == False #node with id 0 does not exist

assert len(graph_banana.verify_node_connections()) == 0 #all bannanas should be connected

nodes_ids_3 = {1,2,3,4,5,6,7,8,9}
edges = [geo.GraphEdge(1,2), geo.GraphEdge(2,3), geo.GraphEdge(3,1), geo.GraphEdge(1,4), geo.GraphEdge(1,5), geo.GraphEdge(6,7), geo.GraphEdge(7,8), geo.GraphEdge(8,9), geo.GraphEdge(9,6)]

graph_apples = geo.SimpleVisibilityGraph(nodes_ids_3)
for edge in edges:
    graph_apples.try_add_edge(edge)

unconnected_apples = graph_apples.verify_node_connections()

print("unconnected_apples:", unconnected_apples)
assert unconnected_apples == {6,7,8,9} or unconnected_apples == {1,2,3,4,5}

print("testing coordiante conversions")
polar_coords = [(4.5, 0.4, np.pi/6),
(4.5, -0.4, np.pi/6),
(4.5, 0.0, np.pi/6),
(4.5, 0.4, 0.0),
(4.5, 0.0, 0.0)
]


for p in polar_coords:  
    cart_coords = geo.shperical_to_cartesian(p[0], p[1], p[2])
    polar_coords = geo.cartesian_to_spherical(cart_coords[0], cart_coords[1], cart_coords[2])
    for i in (0,1,2):
        print(np.abs(p[i] - polar_coords[i]))



#TODO: test Pose class

print("tests passed")