import kimera_scene_graph_pybind as sg
from os import listdir, path, mkdir, getcwd
from scene_graph_learning.scene_graph import SceneGraph, SceneNode, SceneEdge, NodeType
from scene_graph_learning.scene_graph import add_object_connectivity, remove_layer
import numpy as np
import pickle
import random
import time

LAYER_ID_TO_NODE_TYPE = {
    sg.LayerId.kBuildingsLayerId : NodeType.building,
    sg.LayerId.kObjectsLayerId : NodeType.object,
    sg.LayerId.kRoomsLayerId : NodeType.room,
    sg.LayerId.kPlacesLayerId : NodeType.place
}

PATH_TO_KIMERASG = path.dirname(getcwd())+"/kimera_scene_graph_python/vxblx_files/"
SCENE_GRAPH_FILE_NAME = "goseek_scene_graph_scene4"


DOWNSAMPLED_PLACE_NODE_NUM = 100 #number place nodes to add initially for intra layer connections
MAX_PLACE_NODES = 3000 #maximum number of place nodes
PLACES_DOWNSAMPLED = False
ROOM_ID_TO_SEMANTIC_LABEL = {
    "goseek_scene_graph_scene1": {
        1: "Office",
        2: "Restroom",
        3: "Office",
        4: "Office",
        5: "Hallway",
        6: "Hallway",
        7: "Corner Stall"
    },
    "goseek_scene_graph_scene2": {
        1: "Restroom",
        2: "Office",
        3: "Office",
        4: "Hallway",
        5: "Hallway",
        6: "Office",
        7: "Break Room",
        8: "Corner Stall"
    },
    "goseek_scene_graph_scene3": {
        1: "Storage Room",
        2: "Office",
        3: "Hallway",
        4: "Hallway",
        5: "Office",
        6: "Office",
        7: "Break Room"
    },
    "goseek_scene_graph_scene4": {
        1: "Cubical Space",
        2: "Storage Room",
        3: "Hallway",
        4: "Office",
        5: "Office"
    },
    "goseek_scene_graph_scene5": {
        1: "Cubical Space",
        2: "Office",
        3: "Hallway",
        4: "Restroom",
        5: "Office",
        6: "Break Room",
        7: ""
    },

}

def single_node_save():
    graph = sg.SceneGraph()
    output_graph = SceneGraph()
    sg.load_scene_graph(os.path.dirname(os.getcwd())+"/kimera_scene_graph_python/vxblx_files/office_scene_graph.vxblx", graph)

    inter_layer_edges = graph.getInterLayerEdgesMap()
    database = graph.getDatabase()
    objects_node_map = database[sg.LayerId.kObjectsLayerId].getNodeIdMap()
    obj_node = objects_node_map[1]
    obj_node_position = obj_node.attributes_.position_
    centroid = np.array([obj_node_position.x,obj_node_position.y,obj_node_position.z])
    obj_node_bb = obj_node.attributes_.bounding_box_
    size_x = obj_node_bb.max_.x - obj_node_bb.min_.x
    size_y = obj_node_bb.max_.y - obj_node_bb.min_.y
    size_z = obj_node_bb.max_.z - obj_node_bb.min_.z
    size = np.array([size_x, size_y, size_z])
    semantic_label = obj_node.attributes_.semantic_label_
    object_node_1 = SceneNode(1, NodeType.object, centroid, size, semantic_label)
    output_graph.add_node(object_node_1)
    with open('pickled_output_graphs/test_single_node' + '.pkl', 'wb') as output:
        pickle.dump(output_graph, output, protocol=2)

def parse_scene_graph(kimerasg):
    """
    Parse kimera-serialized scene graph data to SceneGraph class object
    """
    database = kimerasg.getDatabase()
    output_graph = SceneGraph()

    #STEP ONE: populate nodes by layer
    ## Building Nodes
    print("STARTED ADDING ROOM AND BUILDING NODES")
    building_layer = database[sg.LayerId.kBuildingsLayerId]
    building_layer_nodes = building_layer.getNodeIdMap()
    for building_id in building_layer_nodes:
        #create the node
        building_node_pos = building_layer_nodes[building_id].attributes_.position_
        centroid = np.array([building_node_pos.x,building_node_pos.y,building_node_pos.z])
        size = 0 #dont have building size
        semantic_label = "building" #TODO: we don't have name, need to make param
        building_node = SceneNode(building_id, NodeType.building, centroid, size, semantic_label)
        output_graph.add_node(building_node)

    room_ids_to_skip = []
    ## Room Nodes and room -> room edges
    room_layer = database[sg.LayerId.kRoomsLayerId]
    room_layer_nodes = room_layer.getNodeIdMap()
    for room_id in room_layer_nodes:
        room_node_pos = room_layer_nodes[room_id].attributes_.position_
        centroid = np.array([room_node_pos.x,room_node_pos.y,room_node_pos.z])
        size = 0 #dont have room size
        if SCENE_GRAPH_FILE_NAME in ROOM_ID_TO_SEMANTIC_LABEL:
            semantic_label = ROOM_ID_TO_SEMANTIC_LABEL[SCENE_GRAPH_FILE_NAME][room_id] #"room"
            if semantic_label == "":
                #skip this room label, remove all edges going to/from this room_node
                room_ids_to_skip.append(room_id)
                print("skipping room, ", room_id)
                continue
        else:
            semantic_label = "room"
        room_node = SceneNode(room_id, NodeType.room, centroid, size, semantic_label)
        output_graph.add_node(room_node)
    room_intra_edges = room_layer.getEdgeIdMap()
    for edge_id, edge in room_intra_edges.items():
        if edge.start_node_id_ in room_ids_to_skip or edge.end_node_id_ in room_ids_to_skip:
            continue
        start_node = output_graph.get_node_by_id_type(edge.start_node_id_, NodeType.room)
        end_node = output_graph.get_node_by_id_type(edge.end_node_id_, NodeType.room)
        r_r_edge = SceneEdge(start_node, 'LocatedNear', end_node)
        output_graph.add_edge(r_r_edge)

    print("FINISHED ADDING ROOM AND BUILDING NODES")

    ## Places Nodes
    print("STARTED ADDING PLACES NODES")
    place_ids_added = set()
    place_layer = database[sg.LayerId.kPlacesLayerId]
    place_layer_nodes = place_layer.getNodeIdMap()
    #Place nodes can be very dense (thousands), so downsample for speed
    if PLACES_DOWNSAMPLED:
        downsampled_place_nodes = random.sample(place_layer_nodes.items(), DOWNSAMPLED_PLACE_NODE_NUM)
        downsampled_place_ids = [place_id for place_id,_ in downsampled_place_nodes]
    else:
        downsampled_place_nodes = place_layer_nodes.items()
        downsampled_place_ids = place_layer_nodes.keys()
    print("number of downsampled place nodes", len(downsampled_place_nodes))
    for place_id, place in downsampled_place_nodes:
        place_node_pos = place_layer_nodes[place_id].attributes_.position_
        centroid = np.array([place_node_pos.x,place_node_pos.y,place_node_pos.z])
        size = 0 #places have no size
        semantic_label = "place"
        place_node = SceneNode(place_id, NodeType.place, centroid, size, semantic_label)
        output_graph.add_node(place_node)
        place_ids_added.add(place_id)
    print("FINISHED ADDING PLACES NODES")

    print("START FINDING PLACE INTRA LAYER EDGES")
    place_intra_edges = place_layer.getEdgeIdMap()
    print("count number of intra place edges", len(place_intra_edges))
    count_edges_in_downsample = 0 #edges that are in the set
    for edge_id, edge in place_intra_edges.items():
        #most of these edges are just skipped when the downsampled set of places are small ~100
        if edge.start_node_id_ in downsampled_place_ids and edge.end_node_id_ in downsampled_place_ids:
            count_edges_in_downsample +=1
            if count_edges_in_downsample%400 == 0:
                print("count now at: ", count_edges_in_downsample)
            start_node = output_graph.get_node_by_id_type(edge.start_node_id_, NodeType.place)
            end_node = output_graph.get_node_by_id_type(edge.end_node_id_, NodeType.place)
            p_p_edge = SceneEdge(start_node, 'Near', end_node)
            output_graph.add_edge(p_p_edge)

    print("final count at: ", count_edges_in_downsample)
    print("FINISHED FINDING PLACE INTRA LAYER EDGES")

    print("START FINDING OBJECT NODES")
    ## Object Nodes
    object_layer = database[sg.LayerId.kObjectsLayerId]
    obj_layer_nodes = object_layer.getNodeIdMap()
    for obj_id, obj_node in obj_layer_nodes.items():
        #create the node
        obj_node_pos = obj_node.attributes_.position_
        centroid = np.array([obj_node_pos.x,obj_node_pos.y,obj_node_pos.z])
        obj_node_bb = obj_node.attributes_.bounding_box_
        size_x = obj_node_bb.max_.x - obj_node_bb.min_.x
        size_y = obj_node_bb.max_.y - obj_node_bb.min_.y
        size_z = obj_node_bb.max_.z - obj_node_bb.min_.z
        size = np.array([size_x, size_y, size_z])
        semantic_label = obj_node.attributes_.semantic_label_ #TODO: get id to string conversion
        object_node = SceneNode(obj_id, NodeType.object, centroid, size, semantic_label)
        output_graph.add_node(object_node)
    # note there are no relationships between objects in kimera scene graphs
    # object_intra_edges = object_layer.getEdgeIdMap() = {}
    print("FINISHED FINDING OBJECT NODES")

    print("START FINDING INTER LAYER EDGES")
    #STEP TWO: populate inter-layer edges (between different layers)
    ## building -> room -> place -> object
    #shuffle the edges, useful if we downsample place nodes
    shuffled_inter_edges = random.sample(kimerasg.getInterLayerEdgesMap().items(), len(kimerasg.getInterLayerEdgesMap().keys()))
    for edge_id, edge in shuffled_inter_edges:
        start_layer = LAYER_ID_TO_NODE_TYPE[edge.start_layer_id_]
        end_layer = LAYER_ID_TO_NODE_TYPE[edge.end_layer_id_]
        if start_layer == NodeType.room and edge.start_node_id_ in room_ids_to_skip:
            continue
        if end_layer == NodeType.room and edge.end_node_id_ in room_ids_to_skip:
            continue
        #following makes sure only inter layer edges to/from place are getting dropped, necessary when too many places
        if (start_layer == NodeType.place or end_layer == NodeType.place) and len(place_ids_added) >= MAX_PLACE_NODES:
            continue
        elif (start_layer == NodeType.place or end_layer == NodeType.place):
            node_id = edge.start_node_id_ if start_layer == NodeType.place else edge.end_node_id_
            if not node_id in place_ids_added:
                print("detected a new place node, inserting now")
                #never enter this loop if all place nodes initially inserted
                insert_place_node(node_id, database, output_graph)
                place_ids_added.add(node_id)
                if len(place_ids_added)%100 == 0:
                    print("total place nodes passed" + str(len(place_ids_added)))

        #if the edges don't connect to places, add them
        start_node = output_graph.get_node_by_id_type(edge.start_node_id_, start_layer)
        end_node = output_graph.get_node_by_id_type(edge.end_node_id_, end_layer)
        o_g_edge = SceneEdge(end_node, 'AtLocation', start_node)
        output_graph.add_edge(o_g_edge)


        #TODO: put this in a test
            # print("ATTEMPTING TO FIND END NODE'S PARENT")
            # parent_idx = output_graph.find_parent_idx(end_node)
            # print("start node, ", start_node)
            # print("parent node, ", output_graph.get_node(parent_idx))
            # assert start_node == output_graph.get_node(parent_idx)
    print("FINISHED FINDING INTER LAYER EDGES")
    print("total place node count" + str(len(place_ids_added)))
    return output_graph

def insert_place_node(node_id, database, output_graph):
    found_place_node = database[sg.LayerId.kPlacesLayerId].getNodeIdMap()[node_id]
    place_node_pos = found_place_node.attributes_.position_
    centroid = np.array([place_node_pos.x,place_node_pos.y,place_node_pos.z])
    size = 0 #places have no size
    semantic_label = "place"
    place_node = SceneNode(node_id, NodeType.place, centroid, size, semantic_label)
    output_graph.add_node(place_node)
    return place_node

def get_node(id, node_type, graph):
    """

    :param id:
    :param node_type:
    :param graph:
    :return:
    """
    return filter(lambda x: x.node_id == id and x.node_type == node_type, graph.get_nodes_copy())[0]

def load_kimerasg():
    ##Loading the graph
    kimerasg = sg.SceneGraph()
    filepath = PATH_TO_KIMERASG + SCENE_GRAPH_FILE_NAME + '.vxblx'
    print(filepath)
    sg.load_scene_graph(filepath, kimerasg)
    return kimerasg

#TODO: Put this in a separate file
def test_parse_graph(output_graph):
    # assert_equals(sg.add(1,2), 3)
    kimerasg = load_kimerasg()
    database = kimerasg.getDatabase()

    # check number of nodes
    input_building_nodes = database[sg.LayerId.kBuildingsLayerId].getNodeIdMap()
    input_room_nodes = database[sg.LayerId.kRoomsLayerId].getNodeIdMap()
    input_place_nodes = database[sg.LayerId.kPlacesLayerId].getNodeIdMap()
    input_object_nodes = database[sg.LayerId.kObjectsLayerId].getNodeIdMap()
    assert output_graph.num_nodes(NodeType.building) == len(input_building_nodes)
    assert output_graph.num_nodes(NodeType.room) == len(input_room_nodes)
    assert output_graph.num_nodes(NodeType.place) == len(input_place_nodes)
    assert output_graph.num_nodes(NodeType.object) == len(input_object_nodes)
    assert output_graph.num_nodes() == len(input_building_nodes) + len(input_room_nodes) + len(input_place_nodes) + len(input_object_nodes)


    # check number and type of edges (there are duplicates in kimerasg)
    inter_layer_edges = kimerasg.getInterLayerEdgesMap()
    room_intra_edges = database[sg.LayerId.kRoomsLayerId].getEdgeIdMap()
    room_intra_edges_without_duplicates = set(map(lambda edge: (edge.start_node_id_, edge.end_node_id_),room_intra_edges.values()))
    place_intra_edges = database[sg.LayerId.kPlacesLayerId].getEdgeIdMap()
    place_intra_edges_without_duplicates = set(map(lambda edge: (edge.start_node_id_, edge.end_node_id_),place_intra_edges.values()))
    object_intra_edges = database[sg.LayerId.kObjectsLayerId].getEdgeIdMap()
    object_intra_edges_without_duplicates = set(map(lambda edge: (edge.start_node_id_, edge.end_node_id_),object_intra_edges.values()))
    expected_total_edges = output_graph.num_edges()
    actual_total_edges = len(inter_layer_edges) + len(room_intra_edges_without_duplicates) + len(place_intra_edges_without_duplicates) + len(object_intra_edges_without_duplicates)
    assert expected_total_edges == actual_total_edges

    print("ALL TESTS PASSED")


if __name__ == "__main__":
    # single_node_save()
    kimerasg = load_kimerasg()
    start_time = time.time()
    output_graph = parse_scene_graph(kimerasg)

    # test_parse_graph(output_graph) ######TODO: figure out how to configure tests in catkin package

    add_object_connectivity(output_graph, threshold_near=2.5, threshold_on=1.0)
    end_time = time.time()
    print("parse_scene_graph_time" + str(start_time-end_time))
    with open('pickled_output_graphs/' + SCENE_GRAPH_FILE_NAME + '_BIG_REFACTOR.pkl', 'wb') as output:
        pickle.dump(output_graph, output, protocol=2)
    print("begun removing layer")
    graph_without_places = remove_layer(output_graph, NodeType.place)
    print("finished removing layer")
    with open('pickled_output_graphs/' + SCENE_GRAPH_FILE_NAME + '_no_places_BIG_REFACTOR.pkl', 'wb') as output:
        pickle.dump(graph_without_places, output, protocol=2)

