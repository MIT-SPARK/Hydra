"""Habitat-specific simulator."""

import json
import pathlib

import click
import networkx as nx
import numpy as np
import spark_dataset_interfaces as sdi

# import spark_dsg.mp3d
import tqdm


def _add_weights(G):
    for s, t in G.edges():
        p_s = np.array(G.nodes[s]["pos"])
        p_t = np.array(G.nodes[t]["pos"])
        G[s][t]["weight"] = np.linalg.norm(p_s - p_t)


def _simplify_faces(G):
    def _update_adjacency(G, adjacency, orig_idx, face_idx):
        if orig_idx not in adjacency:
            adjacency[orig_idx] = [face_idx]
        else:
            neighbors = adjacency[orig_idx]
            p_curr = G.nodes[face_idx]["pos"]
            for n in neighbors:
                if not G.has_edge(face_idx, n):
                    p_n = G.nodes[n]["pos"]
                    G.add_edge(face_idx, n, weight=np.linalg.norm(p_curr - p_n))

            adjacency[orig_idx].append(face_idx)

    G_simple = nx.Graph()
    faces = [x for x in nx.simple_cycles(G, 3)]
    adjacency = {}
    for face_idx, face in enumerate(faces):
        if len(face) < 3:
            click.secho("Invalid face: {face_idx} -> {face}")
            continue

        p1 = G.nodes[face[0]]["pos"]
        p2 = G.nodes[face[1]]["pos"]
        p3 = G.nodes[face[2]]["pos"]
        p_f = np.mean([p1, p2, p3], axis=0)
        G_simple.add_node(face_idx, pos=p_f)
        _update_adjacency(G_simple, adjacency, face[0], face_idx)
        _update_adjacency(G_simple, adjacency, face[1], face_idx)
        _update_adjacency(G_simple, adjacency, face[2], face_idx)

    return G_simple


def _cluster_spatial(G_orig, tolerance):
    G = G_orig.copy()
    merges = {}
    for x in G:
        if x in merges:
            continue

        for n in G[x]:
            if n < x:
                continue

            dist = np.linalg.norm(G.nodes[x]["pos"] - G.nodes[n]["pos"])
            if dist > tolerance:
                continue

            merges[n] = x

    for child, parent in merges.items():
        nx.contracted_nodes(G, parent, child, self_loops=False, copy=False)

    _add_weights(G)
    return nx.convert_node_labels_to_integers(G)


def _prune_tree_leaves(G):
    _add_weights(G)
    T = nx.minimum_spanning_tree(G)
    leaves = []
    for n in T:
        if len(T[n]) == 1:
            leaves.append(n)

    for leaf in leaves:
        T.remove_node(leaf)

    _add_weights(T)
    return nx.convert_node_labels_to_integers(T)


def _get_room_nodes(G, rooms, z_tol=0.25, mask_distance=10000.0):
    p_w = np.zeros((len(G), 3))
    for x in G:
        p_w[x, :] = np.squeeze(G.nodes[x]["pos"])

    room_nodes = []
    for room in rooms:
        dists = np.linalg.norm((p_w - room.centroid)[:, :2], axis=1)
        mask = np.array(
            [not room.pos_inside_room(x + np.array([0, 0, z_tol])) for x in p_w]
        )
        dists[mask] = mask_distance
        room_nodes.append(np.argmin(dists))

    return room_nodes


def _load_graph(filepath):
    with open(filepath) as fin:
        orig = nx.node_link_graph(json.load(fin))

    largest_cc = max(nx.connected_components(orig), key=len)
    G = nx.convert_node_labels_to_integers(orig.subgraph(largest_cc))

    w_R_h = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    for idx in G:
        G.nodes[idx]["pos"] = np.squeeze(w_R_h @ np.array(G.nodes[idx]["pos"]))

    return G


def get_room_trajectory(
    habitat_path, graph_path, cluster_threshold_m=0.5, z_offset=0.5, **kwargs
):
    """
    Get a trajectory that explores the entire scene.

    The general procedure is:
        - Get a graph from the navmesh at a reasonable resolution
        - Compress faces of the navmesh to be nodes (roughly the dual of the mesh)
        - Spatially cluster nearby nodes to remove small faces
        - Compute the minimum spanning tree and drop any leaves to get a backbone
        - Get the shortest path that explores all nodes via traveling salesman
        - (TBD: add rotations at each room)

    Args:
        habitat_path (os.PathLike): Path to habitat scenes
        graph_path (os.PathLike): Graph to load
        cluster_threshold_m (float): Distance threshold for clustering nearby nodes
        z_offset (float): Agent height offset

    Returns:
        Trajectory: Trajectory that covers every node in sparsified navigational graph

    """
    G = _load_graph(graph_path)
    G = _simplify_faces(G)
    if cluster_threshold_m > 0.0:
        G = _cluster_spatial(G, cluster_threshold_m)

    G = _prune_tree_leaves(G)
    path = nx.approximation.traveling_salesman_problem(G, cycle=False)

    # house_path = habitat_path / graph_path.stem / f"{graph_path.stem}.house"
    # mp3d_info = spark_dsg.mp3d.load_mp3d_info(house_path)
    # rooms = spark_dsg.mp3d.get_rooms_from_mp3d_info(mp3d_info, angle_deg=-90)
    # room_nodes = _get_room_nodes(G, rooms)

    b_R_c = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    delta_z = np.array([0, 0, z_offset])
    p_b = np.array([G.nodes[x]["pos"] + delta_z for x in path])
    return sdi.Trajectory.from_positions(p_b, body_R_camera=b_R_c, **kwargs)


@click.command()
@click.argument("habitat_path", type=click.Path(exists=True))
@click.argument("graphs", type=click.Path(exists=True), nargs=-1)
@click.option("--output", "-o", type=click.Path(), default=None)
@click.option("--z-offset", "-z", default=0.6, type=float)
def main(habitat_path, graphs, output, z_offset):
    habitat_path = pathlib.Path(habitat_path).expanduser().absolute()
    graphs = [pathlib.Path(x).expanduser().absolute() for x in graphs]

    if output is None:
        output = pathlib.Path(".").absolute()
    else:
        output = pathlib.Path(output).expanduser().absolute()

    output.mkdir(exist_ok=True, parents=True)

    for graph in tqdm.tqdm(graphs):
        try:
            trajectory = get_room_trajectory(habitat_path, graph, z_offset=z_offset)
        except Exception as e:
            click.secho(f"{graph.stem} failed: {e}")

        trajectory.to_csv(output / f"{graph.stem}.csv")


if __name__ == "__main__":
    main()
