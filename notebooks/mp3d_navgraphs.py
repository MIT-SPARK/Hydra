# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.16.6
#   kernelspec:
#     display_name: Python 3 (ipykernel)
#     language: python
#     name: python3
# ---

# %%
import json
import pathlib

import distinctipy
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import shapely.plotting
import spark_dsg.mp3d

# %matplotlib widget


# %%
scene_name = "q9vSo1VnCiC"


# %%
def load_graph(filepath):
    with open(filepath) as fin:
        orig = nx.node_link_graph(json.load(fin))

    largest_cc = max(nx.connected_components(orig), key=len)
    G = nx.convert_node_labels_to_integers(orig.subgraph(largest_cc))
    print(G)

    w_R_h = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    for idx in G:
        G.nodes[idx]["pos"] = np.squeeze(w_R_h @ np.array(G.nodes[idx]["pos"]))

    return G


graph_path = f"/data/datasets/mp3d_navgraphs/{scene_name}.json"
graph_path = pathlib.Path(graph_path).expanduser().absolute()
G = load_graph(graph_path)


# %%

mp3d_info = spark_dsg.mp3d.load_mp3d_info(
    f"/data/datasets/habitat/mp3d/{scene_name}/{scene_name}.house"
)
rooms = spark_dsg.mp3d.get_rooms_from_mp3d_info(mp3d_info, angle_deg=-90)


# %%
def _add_weights(G):
    for s, t in G.edges():
        p_s = np.array(G.nodes[s]["pos"])
        p_t = np.array(G.nodes[t]["pos"])
        G[s][t]["weight"] = np.linalg.norm(p_s - p_t)


def _update_adjacency(G, adjacency, orig_idx, face_idx):
    if orig_idx not in adjacency:
        adjacency[orig_idx] = [face_idx]
    else:
        neighbors = adjacency[orig_idx]
        for n in neighbors:
            if not G.has_edge(face_idx, n):
                G.add_edge(
                    face_idx,
                    n,
                    weight=np.linalg.norm(G.nodes[face_idx]["pos"] - G.nodes[n]["pos"]),
                )

        adjacency[orig_idx].append(face_idx)


def _simplify_faces(G):
    G_simple = nx.Graph()
    faces = [x for x in nx.simple_cycles(G, 3)]
    adjacency = {}
    for face_idx, face in enumerate(faces):
        p1 = G.nodes[face[0]]["pos"]
        p2 = G.nodes[face[1]]["pos"]
        p3 = G.nodes[face[2]]["pos"]
        p_f = np.mean([p1, p2, p3], axis=0)
        G_simple.add_node(face_idx, pos=p_f)
        _update_adjacency(G_simple, adjacency, face[0], face_idx)
        _update_adjacency(G_simple, adjacency, face[1], face_idx)
        _update_adjacency(G_simple, adjacency, face[2], face_idx)

    return G_simple


def _cluster(G_orig, tolerance):
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


def _prune_tree(G):
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


def _pos_array(G):
    p_w = np.zeros((len(G), 3))
    for x in G:
        p_w[x, :] = np.squeeze(G.nodes[x]["pos"])

    return p_w


G = _simplify_faces(G)
G = _cluster(G, 0.5)
G = _prune_tree(G)


# %%
def _get_room_nodes(G, z_tol=0.25):
    p_w = _pos_array(G)

    room_nodes = []
    for room in rooms:
        dists = np.linalg.norm((p_w - room.centroid)[:, :2], axis=1)
        mask = np.array(
            [not room.pos_inside_room(x + np.array([0, 0, z_tol])) for x in p_w]
        )
        dists[mask] = 100.0
        room_nodes.append(np.argmin(dists))

    return room_nodes


room_nodes = _get_room_nodes(G)
print(room_nodes)

# %%
plt.close("all")
fig, ax = plt.subplots()

colors = distinctipy.get_colors(len(rooms), pastel_factor=0.7)
room_x = [room.centroid[0] for room in rooms]
room_y = [room.centroid[1] for room in rooms]

for idx, room in enumerate(rooms):
    shapely.plotting.plot_polygon(
        room.get_polygon_xy(), ax=ax, color=colors[idx], add_points=False, alpha=0.4
    )

plt.scatter(room_x, room_y, s=100, color=colors, marker="+")

pos = {x: np.squeeze(G.nodes[x]["pos"])[:2] for x in G}
nx.draw_networkx_nodes(G, pos, ax=ax, node_size=20, alpha=0.4)
nx.draw_networkx_nodes(
    G,
    pos,
    room_nodes,
    ax=ax,
    node_size=50,
    node_color=[colors[idx] for idx, _ in enumerate(room_nodes)],
)
nx.draw_networkx_edges(G, pos, ax=ax, alpha=0.3)

ax.set_aspect("equal")
fig.set_size_inches([10, 8])
plt.show()

# %%
path = nx.approximation.traveling_salesman_problem(G, cycle=False)

# %%
fig, ax = plt.subplots()

colors = distinctipy.get_colors(len(rooms), pastel_factor=0.7)
room_x = [room.centroid[0] for room in rooms]
room_y = [room.centroid[1] for room in rooms]

for idx, room in enumerate(rooms):
    shapely.plotting.plot_polygon(
        room.get_polygon_xy(), ax=ax, color=colors[idx], add_points=False, alpha=0.4
    )

plt.scatter(room_x, room_y, s=100, color=colors, marker="+")

p_w = _pos_array(G)
trajectory = np.array([np.squeeze(p_w[x, :2]) for x in path])
indices = np.arange(len(path))
sizes = 50 * indices[::-1] / len(path)
plt.scatter(trajectory[:, 0], trajectory[:, 1], c=indices, s=sizes)

ax.set_aspect("equal")
fig.set_size_inches([10, 8])
plt.show()

# %%
import csv

G_orig = load_graph(graph_path)

points = []
trajectory_path = graph_path.parent / f"{graph_path.stem}.csv"
with trajectory_path.open("r") as fin:
    reader = csv.DictReader(fin)
    for row in reader:
        points.append([float(row["tx"]), float(row["ty"]), float(row["tz"])])

points = np.array(points)

fig, ax = plt.subplots()

colors = distinctipy.get_colors(len(rooms), pastel_factor=0.7)
room_x = [room.centroid[0] for room in rooms]
room_y = [room.centroid[1] for room in rooms]

for idx, room in enumerate(rooms):
    shapely.plotting.plot_polygon(
        room.get_polygon_xy(), ax=ax, color=colors[idx], add_points=False, alpha=0.4
    )

plt.scatter(room_x, room_y, s=100, color=colors, marker="+")
plt.scatter(points[:, 0], points[:, 1], c=np.arange(points.shape[0]), s=2)

pos_orig = {x: np.squeeze(G_orig.nodes[x]["pos"])[:2] for x in G_orig}
nx.draw_networkx_nodes(G_orig, pos_orig, ax=ax, node_size=20, alpha=0.4)
nx.draw_networkx_edges(G_orig, pos_orig, ax=ax, alpha=0.3)

ax.set_aspect("equal")
fig.set_size_inches([10, 8])
plt.show()

# %%
