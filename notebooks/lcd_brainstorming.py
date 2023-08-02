# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.11.5
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# %%
import networkx as nx
import networkx.readwrite as nxio
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
import pandas as pd
import numpy as np
import collections
import shapely
import shapely.geometry
import pathlib
import random
import json

# %matplotlib notebook


# %%
def get_id_parts(node_id):
    """Get human readable components from a node key."""
    key_mask = 0xFF << 56
    key = chr((node_id & key_mask) >> 56)
    idx = node_id & ~key_mask

    return key, idx


def make_id(prefix, index):
    """Make a node key from the node parts."""
    return (ord(prefix) << 56) | (~(0xFF << 56) & index)


def node_matches_prefix(node_id, prefix):
    """Check if an edge matches a prefix."""
    return get_id_parts(node_id)[0] == prefix


def edge_matches_prefix(edge, prefix):
    """Check if an edge matches a prefix."""
    return node_matches_prefix(edge[0], prefix) and node_matches_prefix(edge[1], prefix)


def get_num_nodes(graph, prefix):
    """Get the number of nodes in a layer."""
    return sum((1 for node in graph if node_matches_prefix(node, prefix)))


def load_graph(dsg_path, places_prefix="p"):
    """Open and read graph."""
    with open(dsg_path, "r") as fin:
        data = json.loads(fin.read())

    G = nxio.node_link_graph(
        data,
        attrs={
            "source": "source",
            "target": "target",
            "name": "id",
            "key": "key",
            "link": "edges",
        },
    )

    for node in G:
        if node_matches_prefix(node, places_prefix):
            G.nodes[node]["min_distance"] = G.nodes[node]["attributes"]["distance"]

    for edge in G.edges():
        if edge_matches_prefix(edge, places_prefix):
            G[edge[0]][edge[1]]["min_distance"] = G[edge[0]][edge[1]]["info"]["weight"]

    return G


def get_original_colors(G):
    """Get colors directly from graph."""
    return [
        [float(v) / 255.0 for v in G.nodes[node]["attributes"]["color"]]
        for node in G
        if "color" in G.nodes[node]["attributes"]
    ]


def _clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))


def get_places_colors(G, vmin=0.0, vmax=3.0):
    """Map min distances to a color palette."""
    cmap = sns.color_palette("crest", as_cmap=True)

    colors = []
    for node in G:
        distance = G.nodes[node]["min_distance"]
        clipped_distance = _clamp(distance, vmin, vmax)
        ratio = (clipped_distance - vmin) / (vmax - vmin)
        colors.append(cmap(ratio))

    return colors


def draw_graph(ax, G, color_func=get_original_colors, **kwargs):
    """Draw the graph."""
    positions = {node: G.nodes[node]["attributes"]["position"][:2] for node in G}
    colors = []
    if color_func:
        colors = color_func(G)

    if len(colors) > 0:
        nx.draw(G, positions, ax, node_color=colors, **kwargs)
    else:
        nx.draw(G, positions, ax, **kwargs)


def get_radius_sample(G, seed_node, radius):
    """Sample connected nodes within a distance."""
    seed_pos = np.array(G.nodes[seed_node]["attributes"]["position"])

    visited = set([])
    frontier = collections.deque([seed_node])
    while len(frontier) > 0:
        node = frontier.popleft()
        if node in visited:
            continue

        curr_pos = np.array(G.nodes[node]["attributes"]["position"])
        distance = np.linalg.norm(seed_pos - curr_pos)
        if distance > radius:
            continue

        visited.add(node)

        for neighbor in G.adj[node]:
            frontier.append(neighbor)

    return visited


def plot_scores(ax, score_info, valid_filter, invalid_filter=None, title=None):
    """Separate and plot scores."""
    valid_scores = [s[0] for s in score_info if valid_filter(s)]
    if invalid_filter is None:
        invalid_scores = [s[0] for s in score_info if not valid_filter(s)]
    else:
        invalid_scores = [s[0] for s in score_info if invalid_filter(s)]

    labels = ["overlapping"] * len(valid_scores) + ["non-overlapping"] * len(
        invalid_scores
    )
    scores = valid_scores + invalid_scores

    data = pd.DataFrame({"scores": scores, "labels": labels})

    sns.histplot(
        data,
        x="scores",
        hue="labels",
        ax=ax,
        stat="probability",
        common_norm=False,
        kde=True,
        legend=True,
        hue_order=["overlapping", "non-overlapping"],
    )
    ax.set_xlabel("Cosine Distance")
    if title is not None:
        ax.set_title(title)
    ax.legend(["overlapping", "non-overlapping"])


def make_histogram(distances, N, step, vmin):
    """Make a histogram."""
    counts = [0] * N
    for distance in distances:
        idx = int((distance - vmin) / step)
        idx = _clamp(idx, 0, N - 1)
        counts[idx] += 1

    return np.array(counts).reshape((N, 1))


def histogram_distance(x_a, x_b):
    """Get the cosine distance between two histograms."""
    return np.squeeze(x_a).dot(np.squeeze(x_b)) / (
        np.linalg.norm(x_a) * np.linalg.norm(x_b)
    )


def get_bbox(G, visited):
    """Make a bounding box for a set of notes."""
    x_min = None
    x_max = None
    y_min = None
    y_max = None

    for node in visited:
        pos = G.nodes[node]["attributes"]["position"]
        x_min = pos[0] if x_min is None or pos[0] < x_min else x_min
        x_max = pos[0] if x_max is None or pos[0] > x_max else x_max
        y_min = pos[1] if y_min is None or pos[1] < y_min else y_min
        y_max = pos[1] if y_max is None or pos[1] > y_max else y_max

    return shapely.geometry.box(x_min, y_min, x_max, y_max)


def overlap(G, visited_1, visited_2):
    """Compute the geometric IoU."""
    bbox_1 = get_bbox(G, visited_1)
    bbox_2 = get_bbox(G, visited_2)
    union_area = bbox_1.union(bbox_2).area
    if union_area == 0.0:
        return 0.0

    return bbox_1.intersection(bbox_2).area / union_area


def node_overlap(visited_1, visited_2):
    """Compute the node IoU."""
    if len(visited_1) == 0 and len(visited_2) == 0:
        return 0.0

    return len(visited_1 & visited_2) / len(visited_1 | visited_2)


def show_places_descriptor(score_info, descriptors, hist_midpoints, hist_step):
    """Plot some histograms."""
    # sort by minimum overlap
    score_info.sort(key=lambda x: x[1])
    first_idx = score_info[0][3]
    second_idx = score_info[0][4]

    fig, ax = plt.subplots()
    ax.bar(
        hist_midpoints,
        np.squeeze(descriptors[first_idx]),
        width=-hist_step / 2,
        label="first",
        align="edge",
    )
    ax.bar(
        hist_midpoints,
        np.squeeze(descriptors[second_idx]),
        width=hist_step / 2,
        label="second",
        align="edge",
    )
    fig.set_size_inches([fig_width, 2 * fig_height])
    plt.show()


def draw_sample(ax, node_id, descriptor, hist_info, radius, node_size=30):
    """Draw the sample location and the distance distribution."""
    midpoints, step = hist_info
    ax[1].bar(
        midpoints,
        np.squeeze(descriptor),
        width=step,
    )

    draw_graph(
        ax[0],
        places,
        color_func=lambda x: get_places_colors(x, 0.0, 2.0),
        node_size=node_size,
    )

    highlight_color = [252.0 / 255.0, 186.0 / 255.0, 3.0 / 255.0]
    highlight_edge = [227.0 / 255.0, 169.0 / 255.0, 9.0 / 255.0]
    node_pos = places.nodes[node_id]["attributes"]["position"][:2]
    node_highlight = mpatches.Circle(
        node_pos,
        radius=radius,
        facecolor=highlight_color,
        edgecolor=highlight_edge,
        alpha=0.5,
    )

    ax[0].add_patch(node_highlight)


def score_matches_pairwise(G, samples, descriptors):
    """Compute pairwise scores for all descriptors."""
    score_tuples = []
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            score_tuples.append(
                (
                    histogram_distance(descriptors[i], descriptors[j]),
                    overlap(G, samples[i], samples[j]),
                    node_overlap(samples[i], samples[j]),
                    i,
                    j,
                )
            )
    return score_tuples


# %%
node_size = 30
dsg_dir = "/home/ubuntu/data/saved_dsgs"
dsg_file = "uhumans2_office.json"
dsg_path = pathlib.Path(dsg_dir) / dsg_file
places_prefix = "p"
robot_prefix = "a"
objects_prefix = "O"
rooms_prefix = "R"

G = load_graph(dsg_path, places_prefix)


# %%
print(f"Rooms: {get_num_nodes(G, 'R')}")
print(f"Places: {get_num_nodes(G, 'p')}")
print(f"Objects: {get_num_nodes(G, 'O')}")
print(f"Agents: {get_num_nodes(G, 'a')}")


# %%
robot = nx.subgraph_view(
    G,
    filter_node=lambda x: node_matches_prefix(x, robot_prefix),
    filter_edge=lambda x, y: edge_matches_prefix((x, y), robot_prefix),
)
places = nx.subgraph_view(
    G,
    filter_node=lambda x: node_matches_prefix(x, places_prefix),
    filter_edge=lambda x, y: edge_matches_prefix((x, y), places_prefix),
)
objects = nx.subgraph_view(
    G,
    filter_node=lambda x: node_matches_prefix(x, objects_prefix),
    filter_edge=lambda x, y: edge_matches_prefix((x, y), objects_prefix),
)
rooms = nx.subgraph_view(
    G,
    filter_node=lambda x: node_matches_prefix(x, rooms_prefix),
    filter_edge=lambda x, y: edge_matches_prefix((x, y), rooms_prefix),
)


place_nodes = [n for n in places]
object_nodes = [n for n in objects]


# %%
fig_width = 9.25
fig_height = 4
sns.set()


# %% [markdown]
#
# ## Place Nodes Descriptors
#
# We examine whether a histogram of the minimum distances to an obstacles for a
# set of nodes in the places layer can function as a descriptor.


# %%
places_radius = 5.0

N_hist = 70
hist_dist_min = 0.4
hist_dist_max = 2.0
hist_step = (hist_dist_max - hist_dist_min) / N_hist
hist_midpoints = np.linspace(hist_dist_min, hist_dist_max, N_hist, endpoint=False) + (
    hist_step / 2.0
)


# %%
# ('p', 13261) is hallway coming back
# ('p', 6571) is hallway on first loop?
# ('p', 17342) is hallway coming back
# ('p', 8861) is hallway, about to enter room on right for first loop
# ('p', 31761) is first room when on last pass
# ('p', 2332) is first room on first pass
n1 = make_id("p", 2332)
n1_descriptor = make_histogram(
    [
        places.nodes[n]["min_distance"]
        for n in get_radius_sample(places, n1, places_radius)
    ],
    N_hist,
    hist_step,
    hist_dist_min,
)

n2 = make_id("p", 8861)
n2_descriptor = make_histogram(
    [
        places.nodes[n]["min_distance"]
        for n in get_radius_sample(places, n2, places_radius)
    ],
    N_hist,
    hist_step,
    hist_dist_min,
)

sample_node_size = 10
fig, ax = plt.subplots(2, 2)
draw_sample(
    ax[0],
    n1,
    n1_descriptor,
    (hist_midpoints, hist_step),
    places_radius,
    node_size=sample_node_size,
)
draw_sample(
    ax[1],
    n2,
    n2_descriptor,
    (hist_midpoints, hist_step),
    places_radius,
    node_size=sample_node_size,
)
fig.set_size_inches([fig_width, 2 * fig_height])
plt.show()


# %%
N_samples = 250
sample_nodes = set([random.choice(place_nodes) for _ in range(N_samples)])


# %%

places_samples = [
    (node, get_radius_sample(places, node, places_radius)) for node in sample_nodes
]
places_sets = [s[1] for s in places_samples]
places_distances = [
    [places.nodes[n]["min_distance"] for n in sample] for sample in places_sets
]

places_descriptors = [
    make_histogram(s, N_hist, hist_step, hist_dist_min) for s in places_distances
]
places_scores = score_matches_pairwise(G, places_sets, places_descriptors)


# %%
show_places_descriptor(places_scores, places_descriptors, hist_midpoints, hist_step)


# %%
def unique_overlap(s, min_overlap=0.8, max_node_overlap=0.2):
    """Sample is set-distinct but not geometrically distinct."""
    return s[1] >= min_overlap and s[2] <= max_node_overlap


def unique_nonoverlap(s, min_overlap=0.8, max_node_overlap=0.2):
    """Sample is set-distinct and geometrically distinct."""
    return s[1] < min_overlap and s[2] <= max_node_overlap


min_overlap = 0.7
max_node_overlap = 0.2
fig, ax = plt.subplots()
plot_scores(
    ax,
    places_scores,
    lambda s: unique_overlap(s, min_overlap, max_node_overlap),
    lambda s: unique_nonoverlap(s, min_overlap, max_node_overlap),
    title="Place Descriptor Distributions",
)
fig.set_size_inches([fig_width, 2 * fig_height])
plt.show()


# %%
def make_place_histogram(G, node_sample_pair, distances, N, step, vmin, weight_func):
    """Make a weighted histogram for places."""
    original_pos = np.array(G.nodes[node_sample_pair[0]]["attributes"]["position"])
    node_set = node_sample_pair[1]
    counts = [0] * N
    for node, distance in zip(node_set, distances):
        node_pos = np.array(G.nodes[node]["attributes"]["position"])
        node_dist = np.linalg.norm(original_pos - node_pos)
        weight = weight_func(node_dist)
        idx = int((distance - vmin) / step)
        idx = _clamp(idx, 0, N - 1)
        counts[idx] += weight

    return np.array(counts).reshape((N, 1))


places_descriptors = [
    make_place_histogram(
        G, s, d, N_hist, hist_step, hist_dist_min, lambda x: 1.1 - (x / places_radius)
    )
    for s, d in zip(places_samples, places_distances)
]
weighted_places_scores = score_matches_pairwise(G, places_sets, places_descriptors)


# %%
show_places_descriptor(places_scores, places_descriptors, hist_midpoints, hist_step)


# %%
min_overlap = 0.7
max_node_overlap = 0.2

fig, ax = plt.subplots()
plot_scores(
    ax,
    weighted_places_scores,
    lambda s: unique_overlap(s, min_overlap, max_node_overlap),
    lambda s: unique_nonoverlap(s, min_overlap, max_node_overlap),
    title="Place Descriptor (Weighted Fall-off) Distributions",
)
fig.set_size_inches([fig_width, 2 * fig_height])
plt.show()


# %%
min_overlap = 0.7
max_node_overlap = 0.2

fig, ax = plt.subplots(2, 1, sharex=True)
plot_scores(
    ax[0],
    places_scores,
    lambda s: unique_overlap(s, min_overlap, max_node_overlap),
    lambda s: unique_nonoverlap(s, min_overlap, max_node_overlap),
    title="Place Descriptor",
)
plot_scores(
    ax[1],
    weighted_places_scores,
    lambda s: unique_overlap(s, min_overlap, max_node_overlap),
    lambda s: unique_nonoverlap(s, min_overlap, max_node_overlap),
    title="Weighted Place Descriptor",
)
ax[0].set_ylim([0, 0.4])
ax[1].set_ylim([0, 0.4])
fig.set_size_inches([fig_width, 3 * fig_height])
plt.show()


# %% [markdown]
#
# ## Object Level Descriptors
#
# We show semantic class histograms as descriptors for groups of object nodes


# %%
def get_attached_objects(G, node_set):
    """Get all objects attached to places."""
    objects = set([])
    for node in node_set:
        for neighbor in G.adj[node]:
            if node_matches_prefix(neighbor, "O"):
                objects.add(neighbor)

    return objects


def make_semantic_histogram(G, node_set, N_classes):
    """Make a histogram of semantic classes for objects."""
    scores = [0] * N_classes
    for node in node_set:
        class_idx = int(G.nodes[node]["attributes"]["semantic_label"])
        scores[class_idx] += 1

    return np.array(scores).reshape((N_classes, 1))


object_radius = 5.0
object_classes = 21
min_l1_descriptor_norm = 1
object_sample_sets = [
    get_radius_sample(places, node, object_radius) for node in sample_nodes
]
object_sets = [get_attached_objects(G, sample) for sample in object_sample_sets]
object_descriptors = [
    make_semantic_histogram(G, s, object_classes) for s in object_sets
]

filtered_objects = []
filtered_object_descriptors = []
for i, desc in enumerate(object_descriptors):
    if np.linalg.norm(desc, ord=1) < min_l1_descriptor_norm:
        continue

    filtered_objects.append(object_sets[i])
    filtered_object_descriptors.append(object_descriptors[i])

object_scores = score_matches_pairwise(G, filtered_objects, filtered_object_descriptors)


# %%
min_overlap = 0.6
max_node_overlap = 0.2
fig, ax = plt.subplots()
plot_scores(
    ax,
    object_scores,
    lambda s: unique_overlap(s, min_overlap, max_node_overlap),
    lambda s: unique_nonoverlap(s, min_overlap, max_node_overlap),
    title="Object Descriptor Distributions",
)
fig.set_size_inches([fig_width, 2 * fig_height])
plt.show()
