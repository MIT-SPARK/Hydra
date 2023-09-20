#!/usr/bin/env python3
# Copyright 2022, Massachusetts Institute of Technology.
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Research was sponsored by the United States Air Force Research Laboratory and
# the United States Air Force Artificial Intelligence Accelerator and was
# accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
# and conclusions contained in this document are those of the authors and should
# not be interpreted as representing the official policies, either expressed or
# implied, of the United States Air Force or the U.S. Government. The U.S.
# Government is authorized to reproduce and distribute reprints for Government
# purposes notwithstanding any copyright notation herein.
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
