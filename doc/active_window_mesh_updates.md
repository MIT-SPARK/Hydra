# Active-window mesh update collation

`ActiveWindowOutput::meshUpdates()` exposes ordered batches. Each batch archives
its listed blocks before applying replacements. Repeated replacements of one
block are collapsed to the latest payload until an archive separates its
lifetimes. Replacements without an archive barrier are processed as one snapshot.
The final payload shares ownership with the collated volumetric map; archived
generations retain mesh data only, not historical TSDF/semantic layers.

The event view preserves block lifetimes, not every transient clustering result
from skipped sensor updates. Consumers must not mutate block payloads while an
output is in use. The old `archived` field remains a union for other consumers;
mesh consumers must use the ordered batches to distinguish archival and reentry.

The standalone `ActiveWindowOutput.*` tests cover repeated replacements, archive
boundaries, final-map ownership, and already-collated inputs with map cloning.
