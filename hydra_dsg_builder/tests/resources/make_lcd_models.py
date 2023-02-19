#!/usr/bin/env python3
"""Make a simple model."""
import torch
import torch.onnx
import torch.nn as nn
import torch_geometric.nn as pyg_nn
import torch_geometric as pyg
import onnx
import onnxruntime as ort


class SimpleNetwork(nn.Module):
    """Simple network to make testable onnx export."""

    def __init__(self, fdim, pos_in_feature=True):
        """Make the network."""
        super(SimpleNetwork, self).__init__()
        self.pos_in_feature = pos_in_feature
        self.N = fdim

        self.mp = pyg_nn.GCNConv(self.N, self.N)
        state_dict = {
            "lin.weight": torch.eye(self.N, requires_grad=True, dtype=torch.float32),
            "bias": torch.zeros(self.N, requires_grad=True, dtype=torch.float32),
        }
        self.mp.load_state_dict(state_dict)

    def forward(self, x, edge_index, pos=None):
        """Do a forward pass."""
        if self.pos_in_feature:
            return torch.sum(self.mp(x, edge_index), dim=0, keepdim=True)

        dists = torch.norm(pos[edge_index[0], :] - pos[edge_index[1], :], p=2, dim=1)
        return torch.sum(self.mp(x, edge_index, dists), dim=0, keepdim=True)


def export_model(name, model):
    """Export a model."""
    x = torch.ones((6, model.N))
    edge_index = torch.tensor(
        [[1, 2, 3, 4, 5, 0], [0, 1, 2, 3, 4, 5]], dtype=torch.int64
    )
    edge_index = pyg.utils.to_undirected(edge_index)

    model_input = (x, edge_index)

    input_names = ["x", "edge_index"]
    dynamic_axes = {
        "x": {0: "num_nodes"},
        "edge_index": {1: "num_edges"},
    }

    if not model.pos_in_feature:
        pos = torch.cat(
            (
                torch.unsqueeze(torch.tensor([0.0, 2.0, 3.0, 4.0, 3.0, 2.0]), 1),
                torch.zeros((6, 2)),
            ),
            dim=1,
        )
        input_names.append("pos")
        dynamic_axes["pos"] = {0: "num_nodes"}
        model_input = (x, edge_index, pos)

    with torch.no_grad():
        y = model(*model_input)
        torch.onnx.export(
            model,
            model_input,
            name,
            opset_version=17,
            input_names=input_names,
            output_names=["output"],
            dynamic_axes=dynamic_axes,
        )

    model_onnx = onnx.load(name)
    onnx.checker.check_model(model_onnx)

    opts = ort.SessionOptions()
    ort_session = ort.InferenceSession(name, sess_options=opts)
    out = ort_session.run(
        None, {x: y.numpy() for x, y in zip(input_names, model_input)}
    )
    y_ort = torch.tensor(out[0])

    print("")
    print(name)
    print("-" * len(name))
    print(f"Torch ({y.size()}:{y.dtype}): {y}")
    print(f"ONNX ({y_ort.size()}:{y_ort.dtype}): {y_ort}")
    print(f"Diff: {torch.norm(y_ort - y)}")


def main():
    """Export model to onnx."""
    export_model("places.onnx", SimpleNetwork(5))
    export_model("places_pos.onnx", SimpleNetwork(2, pos_in_feature=False))
    export_model("objects.onnx", SimpleNetwork(8))
    export_model("objects_pos.onnx", SimpleNetwork(5, pos_in_feature=False))


if __name__ == "__main__":
    main()
