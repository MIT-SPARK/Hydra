# Debugging crashes when logging to update_merge.jsonl

When `log_update_merge_analysis: true` is set (e.g. for uhumans2_khronos), the frontend serializes every graph update to JSON and buffers it in memory until save, when it writes to the same frontend output folder as other files: `frontend/update_merge.jsonl`. Crashes can be due to memory, serialization, or something else. This guide helps narrow down the cause.

**Exit code -9 (SIGKILL):** Usually means the Linux **OOM killer** terminated the process. Use **light merge logging** to avoid huge allocations: set `frontend.graph_updater.log_merge_light: true` so each record only logs counts and action/target_node_id (no full node attributes or meshes). See section 4.

## 1. See if the crash is in the merge-log path (exceptions)

The merge-log path is now wrapped in a try/catch. If the failure is a **C++ exception** (e.g. `std::bad_alloc`, `nlohmann::json` exception) during serialization or the callback, you will see:

```text
[Hydra Frontend] Merge log serialization or callback failed (sequence N, ts ...): <exception message>
```

and the process **continues** (that one record is skipped). So:

- **If you see that message:** The crash was in the merge-log path; the message (e.g. `std::bad_alloc`) tells you the cause.
- **If the process still exits/crashes** without that message, the failure is likely:
  - a **segfault** or **abort** (e.g. inside nlohmann or the serializer), or
  - **OOM kill** by the kernel, or
  - in another part of the pipeline (only correlated with “logging on”).

## 2. Get a backtrace (segfault / abort)

Run under GDB so you get a backtrace at the crash:

```bash
# Terminal 1: start with GDB (adjust package/executable if needed)
ros2 run hydra_ros hydra_ros_node --ros-args -p use_sim_time:=true \
  --params-file /path/to/your/params.yaml

# Or run the node under GDB:
gdb --args /path/to/hydra_ros_node ... 
(gdb) run
# When it crashes:
(gdb) bt
(gdb) bt full   # with local variables
```

If you use a launch file:

```bash
gdb --args ros2 launch your_pkg your_launch.py
(gdb) run
# after crash:
(gdb) bt
```

Send the **full backtrace** (and, if possible, `bt full`) to see whether the crash is in:

- `to_json` / `Visitor::to` (attribute serialization),
- `log_record.dump()` (nlohmann),
- `merge_log_records_.push_back` (in-memory buffer),
- or elsewhere (e.g. mesh/graph code).

## 3. Check for OOM (out-of-memory) kill

If the kernel kills the process, you won’t see a normal backtrace. Check:

```bash
dmesg | tail -50
# or
journalctl -k -n 100
```

Look for lines like:

```text
Out of memory: Killed process 12345 (hydra_ros_node)
oom-kill:constraint=CONSTRAINT_NONE ...
```

If OOM is the cause, reduce memory use:

- Enable **clear_object_meshes** for the run (if acceptable) so object meshes are cleared before serialization and the log payload is smaller.
- Shorten the run or use a smaller bag to confirm the crash goes away with less data.

## 4. Narrow down by disabling or shrinking logging

- **Turn off merge logging:** Set `log_update_merge_analysis: false`. If the crash **stops**, the problem is in the merge-log path (memory, serialization, or I/O).
- **Use light merge logging (recommended for OOM):** Set `frontend.graph_updater.log_merge_light: true`. Each record then omits full `frontend_nodes_before` and full attributes; it only logs `frontend_nodes_before_count` and per-update `action` / `target_node_id`. This drastically reduces memory and usually prevents OOM. The analysis notebook supports both full and light formats.
- **Shrink payload (full mode only):** In the dataset config (e.g. uhumans2_khronos), set `clear_object_meshes: true` so object meshes are dropped before the frontend; this reduces the size of each log record.

## 5. Where the crash can happen (code-wise)

When merge logging is on, the flow is:

1. **graph_update.cpp**  
   For each layer update it builds a JSON record:  
   - `to_json(node_entry["attributes"], node->attributes())` for every node in the target layer (`frontend_nodes_before`),  
   - then for each update `to_json(upd["incoming_attributes"], *attrs)` and optionally `to_json(upd["target_attributes"], ...)`,  
   - then `log_callback(log_record.dump())`.

2. **graph_builder.cpp**  
   The callback does `merge_log_records_.push_back(json_line)`. On save, the buffer is written to `output/frontend/update_merge.jsonl` (same folder as `dsg.json`, etc.).

So typical failure points:

- **Memory:**  
  - `log_record.dump()` or the `to_json` calls building a huge string.  
  - `push_back` or the string copy.  
  Mitigation: smaller payload (`clear_object_meshes`), and/or “light” logging (no full attributes).

- **Serialization:**  
  - Exception or crash inside `to_json` / attribute serialization (e.g. mesh or voxel data).  
  The try/catch will show exception messages; for segfaults, use GDB as in section 2.


## 6. Quick checklist

| Step | Action |
|------|--------|
| 1 | Run with merge logging on and check logs for: `Merge log serialization or callback failed` → note the exception message. |
| 2 | If it still crashes, run under GDB and run `bt` (and `bt full`) at the crash. |
| 3 | Run `dmesg \| tail` (or `journalctl -k`) and look for OOM kill. |
| 4 | Set `log_update_merge_analysis: false`. If the crash stops, the cause is in the merge-log path. |
| 5 | Try `clear_object_meshes: true` to reduce memory and payload. |

If you can share the exact error message (from the try/catch log line) or the GDB backtrace and the config (merge log and flush settings), that is enough to pinpoint the next fix (e.g. adding a “light” log mode or fixing a specific serialization path).
