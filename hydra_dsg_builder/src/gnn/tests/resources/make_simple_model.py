#!/usr/bin/env python3
"""Make a simple model."""
import torch
import torch.onnx
import torch.nn as nn
import torch_geometric.nn as pyg


class SimpleNetwork(nn.Module):
    """Simple network to make testable onnx export."""

    def __init__(self):
        """Make the network."""
        super(SimpleNetwork, self).__init__()
        self.mp = pyg.GraphConv(2, 2)
        state_dict = {
            "lin_rel.weight": torch.eye(2, requires_grad=True, dtype=torch.float32),
            "lin_rel.bias": torch.zeros(2, requires_grad=True, dtype=torch.float32),
            "lin_root.weight": torch.eye(2, requires_grad=True, dtype=torch.float32),
        }
        self.mp.load_state_dict(state_dict)

    def forward(self, x, edge_index):
        """Do a forward pass."""
        return self.mp(x, edge_index)


def main():
    """Export model to onnx."""

    model = SimpleNetwork()

    x = torch.ones((5, 2))
    edge_index = torch.tensor([[0, 1, 2, 3], [1, 2, 3, 4]], dtype=torch.long)

    y = model(x, edge_index)
    print(y)

    torch.onnx.export(
        model,
        (x, edge_index),
        "model.onnx",
        export_params=True,
        do_constant_folding=True,
        input_names=["x", "edge_index"],
        output_names=["output"],
        dynamic_axes={"x": {0: "num_nodes"}, "edge_index": {1: "num_edges"}},
    )


if __name__ == "__main__":
    main()
