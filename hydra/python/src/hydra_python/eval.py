"""Evaluate graphs against reusable room and place ground truth.

Construct evaluators with ``from_file``, then call ``eval(graph)`` for each graph.
Place evaluation accepts a ``layer_id`` and ``min_basis`` (extra GVD basis points).
Room scores are zero for empty comparisons; place results are invalid when the
reference GVD is empty. Room overlap rows/columns follow sorted ground-truth and
estimated room IDs. Evaluators do not modify the input graph.
"""

from hydra_python._hydra_bindings.eval import (
    PlaceEvaluator,
    PlaceMetrics,
    RoomEvaluator,
    RoomEvaluatorConfig,
    RoomGeometry,
    RoomMetrics,
    score_rooms,
)

__all__ = [
    "PlaceEvaluator",
    "PlaceMetrics",
    "RoomEvaluator",
    "RoomEvaluatorConfig",
    "RoomGeometry",
    "RoomMetrics",
    "score_rooms",
]
