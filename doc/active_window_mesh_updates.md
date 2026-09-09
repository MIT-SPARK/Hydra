# Active-window map update collation

`ActiveWindowOutput::mapUpdates()` exposes ordered batches for TSDF, mesh,
semantic, and tracking blocks. Each batch archives its listed indices before
applying replacements. Repeated replacements in each layer collapse to the latest
payload until an archive separates the block's lifetimes. Replacements without an
archive barrier are processed as one snapshot. Layers may have different block
indices; compaction never discards one layer's update because another was updated.

The final payload shares ownership with the collated volumetric map. Archived
generations retain all available layers. Voxel storage is reused only when no batch
or reader retains the old block; otherwise replacement detaches it first. The
`setMap(const VolumetricMap&)` overload copies all layers, including optional
semantic and tracking layers. `VolumetricMap::updateFrom()` also merges tracking.

The event view preserves block lifetimes, not every transient result from skipped
sensor updates. Consumers must not mutate block payloads while an output is in use.
`meshUpdates()` remains a compatibility projection of the map batches. Block order
within a batch is unspecified. Batch order defines the archive/replacement order.

`map()` contains final state and `archived` contains accumulated archive indices.
Together these fields cannot describe update/archive/reentry history.
`canCollate()` checks whether merging two ordinary outputs would cross such a
boundary in any layer. The frontend leaves a conflicting packet queued until its
current input is dispatched. This keeps existing final-map callbacks correct and
preserves collation for repeated updates and unrelated archives. Direct callers of
`updateFrom()` can still collate across boundaries, but must use ordered batches to
consume those lifetimes correctly.

The `ActiveWindowOutput.*` tests cover repeated replacements, archive/reentry,
independent layer updates, final-map ownership, immutable retained views,
archive-only messages, cloning, already-collated inputs, and frontend collation
eligibility for each layer independently.
