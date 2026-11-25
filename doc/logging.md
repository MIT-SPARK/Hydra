# Logging

Hydra uses spdlog 1.12 or newer with fmt. Use the same spdlog build configuration
throughout an application; its imported CMake target supplies the matching fmt
settings. Configuration logging can also be routed through config_utilities'
optional spdlog backend.

## Module diagnostics

Configurations that own module diagnostics inherit `hydra::VerbosityConfig` and
register that base in `declare_config`. Unset or null `verbosity` inherits the
active run's `default_verbosity` when a message is evaluated. An explicit value
stays fixed across runs, including zero. Levels are cumulative: zero enables
level-zero diagnostics, and a negative threshold suppresses nonnegative levels.
The process fallback is 1.

Use level 1 for lifecycle messages and summaries, level 2 for per-update
output, and level 3 for normal diagnostic details. Reserve level 4 for very
detailed traces and level 5 for data dumps or test diagnostics. Level 0 remains
available for essential messages.

`log_name` is the component name. `log_prefix` and `log_suffix` surround it
and default to `[` and `] `. These fields are independent of node-symbol
`prefix` fields. Labels are hidden by default; set `show_log_prefix: true` in
a module's configuration to include its label. An empty `log_name` also hides it.

Construct configs with `VerbosityConfig("component")`. `with_name` changes the
name while preserving verbosity and delimiters.

```cpp
MLOG(2, "Processed {} nodes", count);
MLOG_WITH(other_config, 2, "Processing {}", node);
MLOG_NAMED(INFO, "Started");
MLOG_NAMED(WARN, "Missing sensor {}", name);
MLOG_NAMED(ERROR, "Failed to load {}", path);
HYDRA_CHECK_EQ(actual, expected, "Unexpected node count");
```

`MLOG` uses the local `config`; `MLOG_WITH` accepts one explicitly. Diagnostic
arguments are evaluated only when the message is enabled. Native fmt formatting
is preferred, with stream operators adapted automatically for other types.
Configurations and Eigen matrices can be logged directly.

`MLOG_NAMED` uses the local `config` and accepts `INFO`, `WARN`, and `ERROR`.
`MLOG_NAMED_WITH` accepts an explicit configuration. Both render the configured
label and follow the severity filter without checking module verbosity.

Warnings and errors bypass module verbosity. `HYDRA_CHECK` and its comparison
variants, `HYDRA_CHECK_NOTNULL`, and `HYDRA_LOG(FATAL, ...)` preserve terminating
invariant failures, including in release builds. Messages retain their original
source file and line, and fatal failures flush before termination. If diagnostic
formatting or logging throws, a fallback diagnostic is attempted and the process
still aborts.

## Application controls

Set `log_to_file: true` to save diagnostics alongside the run's other artifacts:

```yaml
log_path: /path/to/run
log_to_file: true
output: {use_timestamp: true}
```

This top-level configuration works for both Python pipelines and the ROS node.
It appends to `hydra.log` in the allocated run directory, including any timestamp
subdirectory, while keeping console or ROS output. It defaults to false and
requires a nonempty `log_path`. Directory allocation and overwrite rules apply
before the file opens; with `output.allocate: false`, the directory must exist.

Switching runs moves diagnostic file output to the active context's directory.
A run with `log_to_file: false` removes the previous run's file destination.
Reusing a context restores its file destination without clearing its artifacts.
A `save` output override does not move diagnostic logs. Activate runs after the
previous run's workers have stopped.

`log_level` selects a textual minimum severity: `trace`, `debug`, `info`,
`warning`, `error`, `critical`, or `off`. Module diagnostics emit at info severity,
so both this filter and module verbosity apply. `forward_logs_to_ros` selects ROS
forwarding in the Hydra node without duplicate console emission.
Forwarded messages use the node's `spdlog` child logger at ROS debug level, so
`log_level` controls their severity without changing the node's own ROS threshold.
ROS supplies the timestamp, severity, and logger name. The sink passes the original
source filename and line as metadata rather than adding them to the message.
Directory paths are omitted to match file logging. To include the location once,
set `RCUTILS_CONSOLE_OUTPUT_FORMAT` to
`[{severity}] [{name}] [{file_name}:{line_number}] {message}`.

Python exposes `set_log_level("info")`, `set_log_directory(path,
stderr_level="error")`, and `set_log_to_stderr()`. File output appends to
`hydra.log`; changing destinations preserves severity and module verbosity.
C++ applications use the corresponding functions in `hydra::logging`, or supply
native spdlog sinks with `setSinks`.

Destinations are process-wide. Reconfigure them between runs after workers stop.
Reactivating a run context restores its run defaults, including PGMO's logging
enable flag. PGMO uses its own native spdlog logger with the selected sinks;
its debug messages use debug severity.

The glog-specific configuration keys and Python APIs have been removed without
aliases. Use `default_verbosity` instead of the separate global glog verbosity,
`log_name` instead of the old verbosity-config `prefix`, and the application
controls above for severity and destinations. Use `log_prefix` and `log_suffix`
to customize label delimiters. Other dependencies may still use
glog; this migration removes Hydra's direct dependency on it.
