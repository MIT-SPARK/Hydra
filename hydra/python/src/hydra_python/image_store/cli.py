"""``hydra image-store`` command line (also ``python -m hydra_python.image_store``)."""

from __future__ import annotations

import json

import click

from .store import FrameKindError, ImageStore, SchemaVersionError


class _StoreError(click.ClickException):
    exit_code = 2


class _Group(click.Group):
    """Report store format errors (wrong schema version / frame kind) as exit code 2."""

    def invoke(self, ctx):
        try:
            return super().invoke(ctx)
        except (FrameKindError, SchemaVersionError) as err:
            raise _StoreError(str(err)) from err


@click.group(name="image-store", cls=_Group)
def cli():
    """Index Hydra/Khronos image logs into an image store and inspect stores."""


@cli.command()
@click.argument("store", nargs=-1, required=True)
@click.option("--json", "as_json", is_flag=True, help="print the summary as json")
def info(store, as_json):
    """Summarize one or more stores (directory or store.sqlite)."""
    with ImageStore.open(*store) as s:
        summary = s.info()
    if as_json:
        click.echo(json.dumps(summary, indent=2))
        return
    for s in summary:
        click.echo(f"{s['path']}")
        click.echo(
            f"  schema_version {s['meta'].get('schema_version')}  robots {s['robots']}"
            f"  sensors {s['sensors']}"
        )
        frames = ", ".join(f"{k} {v}" for k, v in s["frames"].items()) or "none"
        click.echo(
            f"  frames: {frames}  (with rgb {s['frames_with_rgb']}, "
            f"unanchored {s['frames_unanchored']})"
        )
        click.echo(
            f"  observations {s['observations']} (kept {s['observations_kept']}) "
            f"over {s['tracks']} tracks"
        )
        click.echo(
            f"  track bindings {s['track_bindings']} -> {s['bound_nodes']} nodes, "
            f"node aliases {s['node_aliases']}"
        )
        extra = {k: v for k, v in s["meta"].items() if k != "schema_version"}
        if extra:
            click.echo(f"  meta {extra}")


@cli.command()
@click.argument("store", nargs=-1, required=True)
@click.option(
    "--dsg",
    help="scene graph used to correct poses (json, or anything spark_dsg loads)",
)
@click.option("--out", required=True, help="output directory")
@click.option("--copy", is_flag=True, help="copy files instead of hard/symlinking")
@click.option("--kept-only", is_flag=True, help="only kept observations")
def export(store, dsg, out, copy, kept_only):
    """Write one folder of images and metadata per node."""
    from .export import export_store
    from .poses import load_dsg

    pose_source = load_dsg(dsg) if dsg else None
    with ImageStore.open(*store) as s:
        stats = export_store(s, out, pose_source, copy=copy, kept_only=kept_only)
    click.echo(json.dumps(stats, indent=1))


@cli.command()
@click.argument("store")
@click.option(
    "--dsg", help="also check keyframe anchors / bound nodes against this scene graph"
)
@click.option("--json", "as_json", is_flag=True, help="print the report as json")
@click.pass_context
def check(ctx, store, dsg, as_json):
    """Structural consistency checks of a store (run or store directory).

    Exits 1 when a check fails."""
    from .check import check_store

    report = check_store(store, dsg=dsg)
    if as_json:
        click.echo(json.dumps(report, indent=1, default=str))
    else:
        for name, ok in report["checks"].items():
            click.echo(f"  {'PASS' if ok else 'FAIL'}  {name}")
        click.echo("ALL PASS" if report["ok"] else "FAILURES")
    ctx.exit(0 if report["ok"] else 1)


def index_options(f):
    """Input/store options shared by ``index`` (and ``tail``)."""
    options = [
        click.option(
            "--keyframes", help="KeyframeSelector.log_path (<ts>.zip archives)"
        ),
        click.option(
            "--tracks", help="Khronos track logger log_path (<track_id>/track.json)"
        ),
        click.option("--hydra", help="Hydra log dir holding backend/ and frontend/"),
        click.option(
            "--dsg",
            help="scene graph for anchoring (default: <hydra>/backend/dsg.json)",
        ),
        click.option("--store", help="store directory (default: <run>/image_store)"),
        click.option(
            "--robot-id",
            type=int,
            default=None,
            help="default: the robot_id of the track logs, else the one the store was "
            "indexed with, else of the event logs, else inferred from the scene "
            "graph's agent prefix, else 0",
        ),
        click.option(
            "--agent-prefix", help="agent NodeSymbol prefix (default: 'a' + robot id)"
        ),
        click.option(
            "--max-anchor-dist",
            type=float,
            default=2.0,
            show_default=True,
            help="[m]",
        ),
        click.option(
            "--copy", is_flag=True, help="copy track images (default: hardlink)"
        ),
    ]
    for option in reversed(options):
        f = option(f)
    return f


@cli.command()
@click.argument("run")
@index_options
@click.option(
    "--partial",
    is_flag=True,
    help="Hydra is still running: leave frames newer than the newest agent node "
    "pending",
)
def index(
    run,
    keyframes,
    tracks,
    hydra,
    dsg,
    store,
    robot_id,
    agent_prefix,
    max_anchor_dist,
    copy,
    partial,
):
    """Build/update <RUN>/image_store from Hydra/Khronos image logs.

    Reads keyframe archives, track logs, track_bindings.json and merges.json; inputs
    default to RUN's keyframes/, tracks/ and hydra/ subdirectories."""
    from .indexer import index as run_index

    stats = run_index(
        run,
        keyframes=keyframes,
        tracks=tracks,
        hydra=hydra,
        store=store,
        dsg=dsg,
        robot_id=robot_id,
        agent_prefix=agent_prefix,
        max_anchor_dist_m=max_anchor_dist,
        link="copy" if copy else "hardlink",
        final=not partial,
    )
    click.echo(json.dumps(stats, indent=1))


def main(argv=None, prog_name: str = "hydra image-store") -> int:
    """Run the CLI on ``argv`` and return its exit code (no ``SystemExit``)."""
    try:
        rv = cli.main(args=argv, prog_name=prog_name, standalone_mode=False)
    except click.exceptions.Exit as exc:
        return exc.exit_code
    except click.ClickException as exc:
        exc.show()
        return exc.exit_code
    except click.Abort:
        click.echo("Aborted!", err=True)
        return 1
    return rv if isinstance(rv, int) else 0
