"""Random stuff."""
import spark_dsg as dsg
import pandas as pd
import numpy as np
import gensim.models
import torch_geometric
import torch
import heapq
import yaml


DEFAULT_LAYERS = (dsg.DsgLayers.OBJECTS, dsg.DsgLayers.ROOMS, dsg.DsgLayers.BUILDINGS)
BUILDING_LABELS = ["house", "office_building", "resort"]


def _load_word2vec(model_file):
    return gensim.models.KeyedVectors.load_word2vec_format(model_file, binary=True)


def _get_embedding(model, label, dim=300):
    vec = np.zeros(dim)
    try:
        vec = np.mean(
            [model[s] for s in label.split("_") if s != "of"],
            axis=0,
        )
    except KeyError:
        print(f"{label} cannot be found in pretrained word2vec")

    return torch.from_numpy(vec.astype(np.float32))


def _get_room_labels(room_file):
    room_df = pd.read_csv(
        room_file, usecols=range(2), names=["char", "room_name"], header=None
    )
    return sorted(list(set(room_df["room_name"].to_list())))


def _get_size(node):
    return node.attributes.bounding_box.max - node.attributes.bounding_box.min


def _get_node_masks_and_features(
    G, embeddings, layers=DEFAULT_LAYERS, embedding_size=300
):
    num_infer_nodes = sum([G.get_layer(x).num_nodes() for x in layers])
    masks = {x: torch.zeros(num_infer_nodes, dtype=torch.bool) for x in layers}

    features = []
    id_map = {}
    for node in G.nodes:
        if node.layer not in layers:
            continue

        # len is a proxy for index
        masks[node.layer][len(features)] = True
        id_map[node.id] = len(features)

        label = node.attributes.semantic_label
        # TODO(nathan) conversion from label to actual sgl label
        embedding = embeddings.get(label, np.zeros(embedding_size))
        pos = node.attributes.position
        size = _get_size(node)
        features.append(np.hstack((pos, size, embedding)))

    return masks, id_map, torch.sensor(features, dtype=torch.float32)


def _get_layer_edge_index(masks, edge_idx, l1, l2=None, undirected=False):
    edge_mask = (
        masks[l1][edge_idx].all(0)
        if l2 is None
        else masks[l1][edge_idx[0, :]] & masks[l2][edge_idx[1, :]]
    )

    layer_idx = edge_idx[:, edge_mask]
    if not undirected:
        return layer_idx

    return (
        torch_geometric.utils.to_undirected(layer_idx)
        if edge_mask.any().item()
        else layer_idx
    )


def _add_edge_attr(data):
    edge_index = torch.cat(
        (
            data.object_edge_index,
            data.room_edge_index,
            data.object_room_edge_index,
            data.room_building_edge_index,
            torch.flipud(data.room_building_edge_index),
            torch.flipud(data.object_room_edge_index),
        ),
        1,
    )
    edge_attr_list = [
        torch.tensor([[1, 0, 0, 0, 0, 0]]),
        torch.tensor([[0, 1, 0, 0, 0, 0]]),
        torch.tensor([[0, 0, 1, 0, 0, 0]]),
        torch.tensor([[0, 0, 0, 1, 0, 0]]),
        torch.tensor([[0, 0, 0, 0, 1, 0]]),
        torch.tensor([[0, 0, 0, 0, 0, 1]]),
    ]
    edge_attr = torch.cat(
        [edge_attr_list[0]] * data.object_edge_index.shape[1]
        + [edge_attr_list[1]] * data.room_edge_index.shape[1]
        + [edge_attr_list[2]] * data.object_room_edge_index.shape[1]
        + [edge_attr_list[3]] * data.room_building_edge_index.shape[1]
        + [edge_attr_list[4]] * data.room_building_edge_index.shape[1]
        + [edge_attr_list[5]] * data.object_room_edge_index.shape[1],
        0,
    ).type(torch.float32)
    data.edge_index = edge_index
    data.edge_attr = edge_attr


def _convert_to_torch_data(G, embedding_dict, add_edge_attr=False):
    """Convert a graph to a pytorch tensor."""
    # TODO(nathan) building label gets converted to known label
    # TODO(nathan) all room labels get mapped to "-"
    # TODO(nathan) object labels get mapped from uint8 to names via object_label_df
    # object_label_df.loc[object_label_df["id"] == label, "id_name"].iloc[0]
    masks, id_map, x = _get_node_masks_and_features(G, embedding_dict)

    # fill edge_index
    edge_index = []
    for edge in G.edges:
        if edge.source not in id_map or edge.target not in id_map:
            continue

        # TODO(nathan) sort edges by layer
        edge_index.append([id_map[edge.source], id_map[edge.target]])
    edge_index = torch.tensor(edge_index, dtype=torch.long)

    # TODO(nathan) make this more generic / check how sgl uses data fields
    R_edges = _get_layer_edge_index(
        masks, edge_index, dsg.DsgLayers.ROOMS, undirected=True
    )
    O_edges = _get_layer_edge_index(
        masks, edge_index, dsg.DsgLayers.OBJECTS, undirected=True
    )

    RB_edges = _get_layer_edge_index(
        masks, edge_index, dsg.DsgLayers.ROOMS, dsg.DsgLayers.BUILDINGS
    )
    OR_edges = _get_layer_edge_index(
        masks, edge_index, dsg.DsgLayers.OBJECTS, dsg.DsgLayers.ROOMS
    )

    torch_data = torch_geometric.data.Data(
        x=x,
        object_mask=masks[dsg.DsgLayers.OBJECTS],
        room_mask=masks[dsg.DsgLayers.ROOMS],
        building_mask=masks[dsg.DsgLayers.BUILDINGS],
        room_edge_index=R_edges,
        object_edge_index=O_edges,
        room_building_edge_index=RB_edges,
        object_room_edge_index=OR_edges,
        edge_index=torch.cat((RB_edges, OR_edges, R_edges, O_edges), dim=1),
    )

    if add_edge_attr:
        # TODO(nathan) we might care about edge type conversion here eventually
        _add_edge_attr(torch_data)

    return torch_data


def get_closest_neighbor_from_ids(
    G: dsg.DynamicSceneGraph, input_node: dsg.SceneGraphNode, node_ids: list
) -> (dsg.SceneGraphNode, float):
    all_dist = [
        np.linalg.norm(
            G.get_node(node_id).attributes.position - input_node.attributes.position
        )
        for node_id in node_ids
    ]
    min_dist = min(all_dist)
    closest_node_idx = all_dist.index(min_dist)
    return G.get_node(node_ids[closest_node_idx]), min_dist


def _get_object_room(G, node):
    return None


def _is_on(G, n1, n2, threshold_on=1.0):
    """
    Check whether n1 is "on" n2 or n2 is "on" n1

    Requires that the n1 center is inside n2 on xy plane, and n1 is above
    n2 on z-axis within a threshold (or vice-versa).
    """
    pos1 = G.get_position(n1)
    pos2 = G.get_position(n2)
    size1 = _get_size(n1)
    size2 = _get_size(n2)

    xy_dist = np.abs(pos1[0:2] - pos2[0:2])
    z_dist = np.abs(pos1[2] - pos2[2])
    n1_above_n2 = pos1[2] > pos2[2]
    new_thresh = threshold_on * (n1.size[2] + n2.size[2]) / 2

    if all(xy_dist <= size2[0:2] / 2) and n1_above_n2 and z_dist <= new_thresh:
        return True
    elif all(xy_dist <= size1[0:2] / 2) and not n1_above_n2 and z_dist <= new_thresh:
        return True
    else:
        return False


def _is_above(G, n1, n2, threshold_near=2.0, threshold_on=1.0):
    """
    Check whether n1 is "above" n2 or n2 is "above" n1

    Requires that the n1 center and n2 are nearby on the xy plane, and n1 is above
    n2 on z-axis by an amount greater than a provided threshold.
    """
    pos1 = G.get_position(n1)
    pos2 = G.get_position(n2)
    size1 = _get_size(n1)
    size2 = _get_size(n2)
    near_thresh = (size1 + size2) / 2.0 * threshold_near

    xy_dist = np.abs(pos1[0:2] - pos2[0:2])
    z_dist = np.abs(pos1[2] - pos2[2])
    n1_above_n2 = pos1[2] > pos2[2]
    dist_thresh = threshold_on * (n1.size[2] + n2.size[2]) / 2

    if all(xy_dist <= near_thresh):
        if n1_above_n2 and z_dist > dist_thresh:
            return True
        if not n1_above_n2 and z_dist > dist_thresh:
            return True

    return False


def _is_under(G, n1, n2):
    """
    Check whether n1 is "under" n2 or n2 is "under" n1

    Requires that either n1 or n2 is inside the other node on the xy place and
    that the positions on the z-axis are distinct.
    """
    pos1 = G.get_position(n1)
    pos2 = G.get_position(n2)
    size1 = _get_size(n1)
    size2 = _get_size(n2)

    xy_dist = np.abs(pos1[0:2] - pos2[0:2])

    if all(xy_dist <= size1[0:2] / 2) or all(xy_dist <= size2[0:2] / 2):
        if pos1[2] < pos2[2]:
            return True
        if pos2[2] < pos1[2]:
            return True

    return False


def _is_near(G, n1, n2, threshold_near=2.0, max_near=2.0):
    """
    Check whether n1 is "near" n2 or n2 is "near" n1

    Requires that either n1 or n2 is inside the other node on the xy place and
    that the positions on the z-axis are distinct.
    """
    pos1 = G.get_position(n1)
    pos2 = G.get_position(n2)
    size1 = _get_size(n1)
    size2 = _get_size(n2)

    avg_size = (size1 + size2) / 2.0
    near_thresh = avg_size * threshold_near

    dist = np.abs(pos1 - pos2)

    # [LocatedNear]
    if all(dist <= near_thresh) and all(dist - avg_size <= max_near * np.ones(3)):
        return True

    return False


def _add_object_connectivity(G, threshold_near=2.0, threshold_on=1.0, max_near=2.0):
    """Add some object-object edges."""
    room_to_objects = dict()
    for node in G.get_layer(dsg.DsgLayers.OBJECTS).nodes:
        room_id = _get_object_room(G, node)
        if room_id not in room_to_objects:
            room_to_objects[room_id] = [node]
            continue

        cmp_nodes = room_to_objects[room_id]
        for cmp_node in cmp_nodes:
            is_on = _is_on(G, node, cmp_node, threshold_on=threshold_on)
            is_above = _is_above(
                G,
                node,
                cmp_node,
                threshold_near=threshold_near,
                threshold_on=threshold_on,
            )
            is_under = _is_under(G, node, cmp_node)
            is_near = _is_near(
                G, node, cmp_node, threshold_near=threshold_near, max_near=max_near
            )

            if is_on or is_above or is_under or is_near:
                # TODO(nathan) consider getting direction
                G.add_edge(node.id, cmp_node.id)

        room_to_objects[room_id].append(node)


def _dist(G, n1, n2):
    return np.linalg.norm(G.get_position(n1) - G.get_position(n2))


def _get_closest_neighbor_with_parent(G, x, max_hop=1):
    frontier = heapq.heapify([(_dist(x.id, n), n) for n in x.siblings])
    visited = set([x.id])

    while len(frontier) != 0:
        curr_id, dist = heapq.heappop(frontier)
        visited.insert(curr_id)
        curr = G.get_node(curr_id)

        if curr.has_parent:
            return curr_id, dist

        for s in curr.siblings():
            if s in visited:
                continue

            heapq.heappush((_dist(x.id, s), s))

    return None, 0.0


def _find_room_node(G, pos):
    # TODO(nathan) use bbox library?
    return None


def _add_object_room_edges(G):
    objects = G.get_layer(dsg.DsgLayers.OBJECTS)
    for node in objects.nodes:
        if node.has_parent():
            place = G.get_node(node.parent())
            if place.has_parent():
                G.add_edge(place.parent, node.id)
            else:
                closest_place, _ = _get_closest_neighbor_with_parent(G, place)
                if closest_place is None:
                    continue

                G.add_edge(closest_place.parent, node.id)
        else:
            room_node = _find_room_node(node.centroid, G)
            if room_node is None:
                raise ValueError("failed to find object parent.")

            G.add_edge(room_node, node.id)


def main(label_file, word2vec_file):
    with label_file.open("r") as fin:
        label_mapping = yaml.load(fin.read(), Loader=yaml.SafeLoader())

    word2vec = _load_word2vec(word2vec_file)

    object_labels = [x for _, x in label_mapping]
    object_embeddings = {
        label: _get_embedding(word2vec, label) for label in object_labels
    }
    print(object_embeddings)


def hydra_to_torch(G, embeddings, threshold_near=2.0, threshold_on=1.0, max_near=2.0):
    dsg.add_bounding_boxes_to_layer(G, dsg.DsgLayers.ROOMS)
    dsg.add_bounding_boxes_to_layer(G, dsg.DsgLayers.BUILDINGS)

    _add_object_room_edges(G)
    _add_object_connectivity(
        G, threshold_near=threshold_near, max_near=max_near, threshold_on=threshold_on
    )

    return _convert_to_torch_data(G, embeddings, add_edge_attr=True)
