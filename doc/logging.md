# Logging

Hydra uses spdlog 1.12 or newer with fmt.

## Configuration

Use `log_level` to choose the minimum severity and `default_verbosity` to set
how much diagnostic detail to show. Individual modules can override `verbosity`:

```yaml
log_level: info
default_verbosity: 1
backend:
  verbosity: 3
  show_log_prefix: true
```

An unset module verbosity inherits `default_verbosity`, which defaults to 1.
Each verbosity level includes the lower levels. Use levels 1–3 for summaries,
per-update output, and diagnostic details. Reserve 4 for very detailed traces
and 5 for data dumps or tests. Level 0 is for essential messages.

Module labels are hidden by default. Set `show_log_prefix: true` to show them.
`log_name` chooses the label. `log_prefix` and `log_suffix` surround it and
default to `[` and `] `.

`log_level` accepts `trace`, `debug`, `info`, `warning`, `error`, `critical`, or
`off`. Module diagnostics use info severity, so `log_level` must allow info to
see them. Warnings and errors do not use module verbosity.

## Saving logs

Set `log_to_file: true` to save `hydra.log` alongside the run's other artifacts:

```yaml
log_path: /path/to/run
log_to_file: true
output: {use_timestamp: true}
```

This works for both Python and ROS. Logs are appended in the run directory,
including the timestamp subdirectory when enabled. Console or ROS output stays
active. With `output.allocate: false`, the directory must already exist.

Switching runs changes the log file to the active run's directory. Giving
`save` a different output path does not move the log file.

## ROS output

Set `forward_logs_to_ros: true` to send Hydra logs through ROS. `log_level`
controls which messages are forwarded. To include the source filename and line
number in console output:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{name}] [{file_name}:{line_number}] {message}'
```

## Python controls

Use `hydra_python.set_log_level("debug")` to change severity.
`set_log_directory(path)` writes to a separate log directory, and
`set_log_to_stderr()` returns to console output.

Logging destinations are shared across pipelines in the same application.
Change them between runs after workers have stopped.

## Writing module messages

Include `hydra/utils/verbosity_config.h` for configuration declarations and
`hydra/utils/logging.h` where you write messages.

Add `VerbosityConfig` as a base class and call
`config::base<VerbosityConfig>(config)` in `declare_config`. Give each module a
short snake_case name, such as `VerbosityConfig("mesh_segmenter")`.

```cpp
MLOG(2, "Processed {} nodes", count);
MLOG_NAMED(WARN, "Missing sensor {}", name);
```

`MLOG` uses the local `config`. `MLOG_NAMED` accepts `INFO`, `WARN`, and `ERROR`
and does not check module verbosity. For another configuration, use
`MLOG_WITH` or `MLOG_NAMED_WITH`.

See [logging.h](../hydra/include/hydra/utils/logging.h) for the C++ helpers.
