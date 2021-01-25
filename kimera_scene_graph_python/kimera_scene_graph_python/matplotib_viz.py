from scene_graph_learning.scene_graph.scene_graph import SceneNode, SceneEdge, SceneGraph, NodeType
from copy import deepcopy
import numpy as np
import pickle
from os import path
from mpl_toolkits.mplot3d.proj3d import proj_transform
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.text import Annotation


# Plot style
NODE_TYPE_OFFSET = {
    NodeType.building: 40,
    NodeType.object: 0,
    NodeType.place: 15,
    NodeType.room: 25}

NODE_TYPE_TO_COLOR = {
    NodeType.building: "blue",
    NodeType.object: "red",
    NodeType.place: "green",
    NodeType.room: "olive"}

annotations = []
project_dir = path.join(path.dirname(path.abspath(__file__)), '../../')
pickled_output_graphs_dir = path.join(path.dirname(path.abspath(__file__)), './pickled_output_graphs/')


class Annotation3D(Annotation):
    """Creates Annotation for 3D point xyz"""

    def __init__(self, s, xyz, *args, **kwargs):
        Annotation.__init__(self, s, xy=(0, 0), *args, **kwargs)
        self._verts3d = xyz

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.xy = (xs, ys)
        Annotation.draw(self, renderer)


def annotate3D(ax, s, *args, **kwargs):
    """
    Adds Anotation3D to to Axes3d ax
    :param ax: the 3d axes to add the annotation to
    :param s: the text to add to annotation
    :return: instance of Annotation3D
    """
    tag = Annotation3D(s, *args, **kwargs)
    ax.add_artist(tag)
    return tag


def dist(annot_pt, event):
    """
    Calculates distance between annotation point to mouse position
    :param annot_pt: Annotation3D instance
    :param event: mouse event (which contains mouse position in .x and .xdata)
    :return: the distance between mouse position and annotation point
    """
    # Get projected3d point in 2d data space
    x2, y2 = annot_pt.xy
    # Convert 2d data space to 2d screen space
    x3, y3 = ax.transData.transform((x2, y2))
    return np.sqrt((x3 - event.x) ** 2 + (y3 - event.y) ** 2)


def calc_closest_annot(annotations, event):
    """
    Calculates index of closest annotation to mouse position
    :param annotations: the global list of annotations
    :param event: mouse event (which contains mouse position in .x and .xdata)
    :return: index i where annotations[i] represents the closest annotation to the mouse position
    """
    distances = [dist(annot, event) for annot in annotations]
    closest_idx = np.argmin(distances)
    return closest_idx


def onMouseMotion(event):
    """
    Event that's triggered on mouse movement. Makes closest annotation to mouse pointer visible.
    :param event: mouse event (which contains mouse position in .x and .xdata)
    :return: None
    """
    # first set all annotations to invisible
    for annot in annotations:
        annot.set_visible(False)
    # find the annotation closest to mouse point and set to visible
    closest_idx = calc_closest_annot(annotations, event)
    annotations[closest_idx].set_visible(True)
    fig.canvas.draw()


def load_graph_pkl(file_name):
    """ load scene graph stored in .pkl file """
    try:
        with open(file_name, 'rb') as input:
            scene_graph = pickle.load(input)
        return scene_graph
    except EnvironmentError as e:      # OSError or IOError...
        print(e.errno)
    # except FileNotFoundError:
    #     print('Cannot find {}.'.format(file_name))
    #     return SceneGraph()


def viz_nodes(all_nodes, ax, show_label=False):
    """
    Plots nodes, creates hover annotations and labels.
    :param all_nodes: list of SceneNode instances (Note: pass in copy)
    :param ax: the 3d axes
    :param show_label: boolean to display labels, on default False
    :return: None
    """
    for node in all_nodes:
        # #removing places
        # if node.node_type == NodeType.place or node.node_type == NodeType.place:
        #     continue
        # #removing places

        node_color = NODE_TYPE_TO_COLOR[node.node_type]
        x, y, z = node.centroid
        ax.scatter3D(x, y, z, color=node_color)

        # initialize annotation for hover
        rounded_centroid = np.around(node.centroid, decimals=2)
        rounded_size = np.around(node.size, decimals=2)
        annot = annotate3D(
            ax,
            s='{}\ncentroid: {}\n size: {}'.format(node, rounded_centroid, rounded_size),
            xyz=node.centroid, fontsize=9.5, xytext=(-20, 20),
            textcoords='offset points', ha='right', va='bottom',
            bbox=dict(boxstyle='round,pad=0.5', fc=node_color, alpha=0.7),
            arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0')
        )
        annotations.append(annot)
        annot.set_visible(False)

    # labeling
    if show_label:
        for node in all_nodes:
            # #removing places
            # if node.node_type == NodeType.place or node.node_type == NodeType.place:
            #     continue
            # #removing places

            x, y, z = node.centroid
            ax.text(x, y, z + .05, node.semantic_label, horizontalalignment='center', verticalalignment="baseline",
                    size='smaller', color=NODE_TYPE_TO_COLOR[node.node_type])


def viz_edges(all_edges, ax, show_label=False):
    """
    Plots nodes, creates labels.
    :param all_edges: dictionary of SceneEdges (Note: pass in copy)
    :param ax: the 3d axes
    :param show_label: boolean to display labels, on default False
    :return: None
    """
    for edge_key in all_edges:
        # scene_edge = all_edges[edge_key]
        ##TODO: change in main file
        scene_edge_list = all_edges[edge_key]
        scene_edge = scene_edge_list[0] #just taking first one
        #
        # #removing places
        # if scene_edge.start.node_type == NodeType.place or scene_edge.end.node_type == NodeType.place:
        #     continue
        # #removing places


        st_node = z_offset(scene_edge.start)
        end_node = z_offset(scene_edge.end)
        aligned = np.column_stack((st_node.centroid, end_node.centroid))

        if scene_edge.start.node_type != scene_edge.end.node_type and scene_edge.rel == 'AtLocation':
            ax.plot3D(aligned[0], aligned[1], aligned[2], linestyle=(0, (1, 5)), linewidth=.5, color='black')
        else:
            ax.plot3D(aligned[0], aligned[1], aligned[2], linestyle=(0, ()), linewidth=.5, color='red')
            mid_x, mid_y, mid_z = (end_node.centroid + st_node.centroid) / 2
            # labeling
            if show_label and scene_edge.rel != 'LocatedNear':
                ax.text(mid_x, mid_y, mid_z + .05, scene_edge.rel, horizontalalignment='center',
                        verticalalignment="baseline",
                        size='smaller', color='black')


def z_offset(node):
    """ Takes a node and returns node with centroid z coord offset according to node type """
    node_offset = deepcopy(node)
    node_offset.centroid[2] += NODE_TYPE_OFFSET[node_offset.node_type]
    return node_offset


if __name__ == '__main__':
    fig = plt.figure(num=None, figsize=(12, 6.5))
    ax = plt.axes(projection='3d')
    ax.grid(False)
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    # legend
    color_handles = []
    for scene_node_type in NODE_TYPE_TO_COLOR:
        color_handles.append(mpatches.Patch(color=NODE_TYPE_TO_COLOR[scene_node_type], label=scene_node_type))
    plt.legend(handles=color_handles)

    # sg = load_graph_pkl(project_dir + '/dataset_loader/Stanford_graphs/Leonardo.pkl')
    sg = load_graph_pkl(pickled_output_graphs_dir + "goseek_scene_graph_scene3_no_places.pkl")
    # sg = load_graph_pkl(pickled_output_graphs_dir + "office_scene_graph_esdf_slice_1_3_medium.pkl")

    all_nodes = list(map(lambda node: z_offset(node), sg.get_nodes_copy()))
    viz_nodes(all_nodes, ax, show_label=True)
    viz_edges(sg.get_edge_dict_copy(), ax, show_label=True)

    # note panning with middle cursor only works if you build matplotlib from source
    # https://matplotlib.org/3.3.3/faq/installing_faq.html#install-from-source
    plt.gca().set_position([0, 0, 1, 1])

    # fig.canvas.mpl_connect('motion_notify_event', onMouseMotion)
    plt.show()
