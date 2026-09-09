# Active-window collation performance

Measured on 2026-09-09 in the ROS 2 workspace with GCC, `-O2`, and Hydra built as
`RelWithDebInfo`. The benchmark compares the mesh-only collation implementation
at `c53e5fa2` with the revised implementation. Both link the current volumetric-map
implementation, including its tracking merge fix, to isolate collation overhead.

Each input updates eight blocks with 16³ voxels per block, TSDF, semantic and
tracking layers, and 100 mesh vertices per block. Inputs are constructed before
timing. The timed region includes `updateFrom()` and releasing the consumed input.
Results are medians of five separate processes. Archive cases archive and replace
the same indices; these deliberately exercise direct collation across lifetimes.

| Updates | Archive interval | Mesh-only baseline (ms) | All-layer batches (ms) |
| ---: | ---: | ---: | ---: |
| 32 | None | 22.80 | 22.16 |
| 128 | None | 89.36 | 90.18 |
| 128 | Every 16 updates | 103.95 | 97.28 |
| 128 | Every update | 139.38 | 262.30 |

Repeated updates cost about 0.70 ms per output for this workload, with approximately
1% difference at 128 updates. The modest differences in the first three rows are
not evidence of a consistent speedup. The first implementation allocated fresh
voxel blocks on every replacement and measured roughly 55–70% slower. Reusing
unshared voxel storage removed that overhead while retaining immutable snapshots.

Retaining every historical generation is substantially more expensive. At 128
updates, live allocator bytes after collation were:

| Archive interval | Mesh-only baseline (MiB) | All-layer batches (MiB) |
| --- | ---: | ---: |
| None | 8.65 | 8.65 |
| Every 16 updates | 8.78 | 31.59 |
| Every update | 11.65 | 424.93 |

These measurements use glibc `mallinfo2()` (`uordblks + hblkhd`) and include other
live process allocations. Peak RSS is approximately 481 MiB in most 128-update
runs because all input maps are constructed in advance; it obscures the difference
in retained history and should not be used to estimate batch storage.

Compaction scans retained block references on each append. With no archives,
repeated updates retain one payload per block per layer. With an archive on every
update, history grows linearly and repeated compaction can take quadratic total
metadata work. Voxel payload storage also grows with retained lifetimes. The new
frontend eligibility check prevents such histories from accumulating inside one
collated frontend packet. Its cost is linear in the archive lists, with hash
lookups in each layer. Crossing a lifetime boundary requires another frontend
pass; this benchmark does not measure that pass or establish end-to-end sensor
throughput. A recorded-data replay is needed to quantify the frontend tradeoff.

The workspace directory `benchmark_results/active_window_collation/` contains the
C++ harness, a standalone `run_benchmark.py`, raw timings, allocator measurements,
and the initial allocation-heavy results. After building Hydra without sourcing
the workspace, reproduce with:

```sh
python3 benchmark_results/active_window_collation/run_benchmark.py
```

Validation: `colcon build --packages-select hydra` succeeded;
`colcon test --packages-select hydra` passed all 212 enabled tests (two existing
tests are disabled). `colcon test-result --test-result-base build/hydra --verbose`
reported zero errors and failures.
The dependent `hydra_ros` package also rebuilt successfully against the changed
output header. Neither build sourced the workspace.
`colcon test --packages-select hydra_ros` also passed. Its first sandboxed attempt
could not write the ROS log directory; rerunning with that access succeeded.
