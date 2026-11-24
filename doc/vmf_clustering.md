# vMF region clustering

`VmfClustering` assigns query labels to places and returns connected regions of
each label. Its confidence-weighted propagation follows
[FOUND-IT, Section III-D](https://arxiv.org/html/2605.25371v1#S3.SS4)
and the [reference implementation](https://github.com/MIT-SPARK/FOUND-IT/blob/4836f34c45c2b088fc6c5a6ec60f43c21c40b1d6/foundit/places/clustering/propagation.py).

Use the same encoder for query embeddings and image features. Select
`type: VmfClustering` in the regions updater's `clustering` configuration and
`type: vmf` in the keyframe selector's `feature_selector` configuration.

| Clustering setting | Default | Meaning |
| --- | --- | --- |
| `propagation_strength` | `1` | Multiplier for neighbor influence. |
| `label_propagation_iterations` | `20` | Maximum iterations; `0` keeps initial scores. |
| `convergence_threshold` | `1e-4` | Maximum score change before a query converges. |
| `allow_empty_scores` | `true` | Allow places without features to inherit neighbor evidence. |

Set the feature selector's `kappa_max` explicitly when comparing configurations:
`500` matches the reference setup, while the existing open-regions ROS profile
uses `1e7`. The selector class default is `1e6`.

Propagation anchors each update to the original concentration-weighted cosine
score. Queries converge independently. Components without positive observation
confidence produce no regions; valid isolated places retain their labels. The
regions updater applies its `min_num_nodes` filter after clustering.
