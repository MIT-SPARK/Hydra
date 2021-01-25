import kimera_scene_graph_pybind as sg
import os
import numpy as np

def load_graph_manual_test():
    sum = sg.add(1,2)

    #check constructors don't throw segfault
    edge = sg.SceneGraphEdge()
    node = sg.SceneGraphNode()
    color = sg.NodeColor()
    color_init = sg.NodeColor(100,100,100)
    attributes = sg.NodeAttributes()
    layer = sg.SceneGraphLayer() # segfault due to eigen aligned operator - got rid of macro
    graph = sg.SceneGraph()

    sg.load_scene_graph(os.path.dirname(os.getcwd())+"/kimera_scene_graph_python/vxblx_files/goseek_scene_graph_scene1.vxblx", graph)
    inter_layer_edges = graph.getInterLayerEdgesMap()
    database = graph.getDatabase()
    # print("database \n" + str(database))
    # print("database inter layer edge map: " + str(len(graph.getInterLayerEdgesMap())))
    # for edge_id, edge in graph.getInterLayerEdgesMap().items():
    #     if edge.start_layer_id_ == sg.LayerId.kBuildingsLayerId:
    #         print(edge)

    # building_node = database[sg.LayerId.kBuildingsLayerId].getNodeIdMap()[1]
    # print("building_node attribute: " + str(building_node.attributes_))
    # print("building_node children_edge_map_: " + str(building_node.children_edge_map_))
    #
    # places_node_map = database[sg.LayerId.kPlacesLayerId].getNodeIdMap()
    # place_node = places_node_map[1]
    # for node_id, place_node in places_node_map.items():
    #     if place_node.children_edge_map_ != {}:
    #         print("found place with children!")
    #         print(place_node)
    #         break
    # print("place_node attribute: " + str(place_node.attributes_))
    # print("place node 1's children_edge_map_" + str(place_node.children_edge_map_))

    # for layer in database:
    #     print("layer: " + str(layer))
    #     print(database[layer])
    room_layer = database[sg.LayerId.kRoomsLayerId]
    room_intra_edges = room_layer.getEdgeIdMap()
    start_end_ids = set()
    duplicate_count = 0
    for edge_id, edge in room_intra_edges.items():
        if (edge.start_node_id_,edge.end_node_id_) in start_end_ids:
            # print("there's a duplicate")
            duplicate_count += 1
            # print((edge.start_node_id_,edge.end_node_id_))
        else:
            start_end_ids.add((edge.start_node_id_,edge.end_node_id_))
    print("duplicates %s" % duplicate_count)
    print("total: ", len(room_intra_edges.keys()))
    # rooms_node_map = database[sg.LayerId.kRoomsLayerId].getNodeIdMap()
    # print(rooms_node_map)
    # room_node = rooms_node_map[1]
    # print("attribute: " + str(room_node.attributes_))
    # print("room hasParent: " + str(room_node.hasParent()))
    # print("room children_edge_map_: " + str(room_node.children_edge_map_)) #only maps rooms to places
    # print("room parent_edge_: " + str(room_node.parent_edge_))
    #
    places_node_map = database[sg.LayerId.kPlacesLayerId].getNodeIdMap()
    parentless_place_count = 0
    parentless_place_with_child_count = 0
    place_with_parent_count = 0
    for key,value in places_node_map.items():
        #keys not sequential! for example 14 doesn't exist in goseek1... why
        if not places_node_map[key].hasParent():
            # print("found place with no parent!!")
            parentless_place_count += 1
            # print(places_node_map[key].node_id_)
            if places_node_map[key].children_edge_map_ != {}:
                print("child: ", places_node_map[key].children_edge_map_) #has children
                parentless_place_with_child_count += 1
        else:
            place_with_parent_count += 1
    print("parentless_place_count", parentless_place_count)
    print("parentless_place_with_child_count", parentless_place_with_child_count)
    print("place_with_parent_count", place_with_parent_count)
    objects_node_map = database[sg.LayerId.kObjectsLayerId].getNodeIdMap()
    for i in range(len(objects_node_map)):
        if not objects_node_map[i].hasParent():
            obj_node = objects_node_map[i]
            print("found object with no parent!!")
            print(obj_node)
    print("number of objects", len(objects_node_map))

    obj_node = objects_node_map[41] #24, 95
    # print("bounding_box: " + str(obj_node.attributes_.bounding_box_))
    # print("position: " + str(obj_node.attributes_.position_.x))
    # print("semantic_label: " + str(obj_node.attributes_.semantic_label_))
    print("object hasParent: " + str(obj_node.hasParent())) #false
    print("parent edge: ", obj_node.parent_edge_)
    # print("object siblings_edge_map_: " + str(obj_node.siblings_edge_map_)) #empty
    # # print("object getNodeIdMap: " + str(database[sg.LayerId.kObjectsLayerId].getNodeIdMap()))
    # print("obj getEdgeIdMap: " + str(database[sg.LayerId.kObjectsLayerId].getEdgeIdMap())) #{} empty
    #

if __name__ == "__main__":
    load_graph_manual_test()
