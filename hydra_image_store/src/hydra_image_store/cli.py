"""``hydra-image-store`` command line."""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

from .store import ImageStore, ImageStoreWriter


def _cmd_info(args) -> int:
    with ImageStore.open(*args.store) as store:
        info = store.info()
    if args.json:
        print(json.dumps(info, indent=2))
        return 0
    for s in info:
        print(f"{s['path']}")
        print(
            f"  schema_version {s['meta'].get('schema_version')}  robots {s['robots']}  sensors {s['sensors']}"
        )
        frames = ", ".join(f"{k} {v}" for k, v in s["frames"].items()) or "none"
        print(
            f"  frames: {frames}  (with rgb {s['frames_with_rgb']}, unanchored {s['frames_unanchored']})"
        )
        print(
            f"  observations {s['observations']} (kept {s['observations_kept']}) over {s['tracks']} tracks"
        )
        print(
            f"  track bindings {s['track_bindings']} -> {s['bound_nodes']} nodes, node aliases {s['node_aliases']}"
        )
        extra = {k: v for k, v in s["meta"].items() if k != "schema_version"}
        if extra:
            print(f"  meta {extra}")
    return 0


def _cmd_sessions(args) -> int:
    from .sessions import list_sessions

    sessions = list_sessions(args.run_dir)
    if args.json:
        out = []
        for session in sessions:
            entry = session.to_json()
            if session.store.joinpath("store.sqlite").exists():
                with session.open() as store:
                    entry["store_info"] = store.info()[0]
            out.append(entry)
        print(json.dumps(out, indent=2))
        return 0
    if not sessions:
        print(f"no image store sessions under {args.run_dir}")
        return 1
    for session in sessions:
        print(f"session {session.name}: {session.store}")
        if session.store.joinpath("store.sqlite").exists():
            with session.open() as store:
                s = store.info()[0]
            frames = ", ".join(f"{k} {v}" for k, v in s["frames"].items()) or "none"
            print(
                f"  frames: {frames}; observations {s['observations']} (kept {s['observations_kept']}); "
                f"bindings {s['track_bindings']}, aliases {s['node_aliases']}"
            )
        dsg = session.dsg_path("backend") or session.dsg_path("frontend")
        print(
            f"  dsg: {dsg if dsg else 'not saved' + (' yet' if session.live else '')}"
        )
        if session.info.get("rotated_at"):
            print(f"  rotated at {session.info['rotated_at']}")
    return 0


def _cmd_export(args) -> int:
    from .export import export_store
    from .poses import load_dsg

    pose_source = load_dsg(args.dsg) if args.dsg else None
    with ImageStore.open(*args.store) as store:
        stats = export_store(
            store, args.out, pose_source, copy=args.copy, kept_only=args.kept_only
        )
    print(json.dumps(stats, indent=1))
    return 0


def _cmd_gc(args) -> int:
    from .gc import gc_store

    with ImageStoreWriter(args.store) as writer:
        stats = gc_store(
            writer,
            args.max_images_per_object,
            orphans=args.orphans,
            dry_run=args.dry_run,
        )
    print(json.dumps(stats, indent=1))
    return 0


def _cmd_import_legacy(args) -> int:
    from .legacy import import_legacy

    link = "copy" if args.copy else ("symlink" if args.symlink else "hardlink")
    stats = import_legacy(
        args.run_dir,
        args.out,
        args.dsg,
        robot_id=args.robot_id,
        link=link,
        sam3_dirs=tuple(args.sam3_dir) if args.sam3_dir else ("sam3_objects",),
    )
    if args.write_dsg:
        from .legacy import find_dsg, strip_legacy_dsg

        src = args.dsg or find_dsg(pathlib.Path(args.run_dir))
        if src is None:
            raise SystemExit(
                "--write-dsg needs --dsg (no scene graph found in the run dir)"
            )
        stats["stripped_dsg"] = strip_legacy_dsg(src, args.write_dsg)
    print(json.dumps(stats, indent=1))
    return 0


def _progress_printer(label: str):
    def progress(done: int, total: int) -> None:
        if done == total or done % 50 == 0:
            print(f"{label} {done}/{total}", file=sys.stderr, flush=True)

    return progress


def _cmd_pack(args) -> int:
    from .pack import pack_store

    stats = pack_store(
        args.run_dir,
        args.out,
        frames=args.frames,
        compression=args.compression,
        zstd_level=args.zstd_level,
        include_masks=not args.no_masks,
        progress=None if args.quiet else _progress_printer("packed"),
    )
    # stdout may carry the archive; report on stderr
    print(json.dumps(stats, indent=1), file=sys.stderr)
    return 0


def _cmd_unpack(args) -> int:
    from .pack import unpack_store

    try:
        stats = unpack_store(
            args.archive,
            args.dest,
            store_name=args.store_name,
            overwrite=args.overwrite,
            check=not args.no_check,
            dsg=args.dsg,
            progress=None if args.quiet else _progress_printer("unpacked"),
        )
    except (RuntimeError, FileExistsError, ValueError) as exc:
        print(f"unpack failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(stats, indent=1, default=str))
    return 0


def _cmd_check(args) -> int:
    from .pack import check_store

    report = check_store(args.store, dsg=args.dsg)
    if args.json:
        print(json.dumps(report, indent=1, default=str))
    else:
        for name, ok in report["checks"].items():
            print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        print("ALL PASS" if report["ok"] else "FAILURES")
    return 0 if report["ok"] else 1


def _cmd_fuse(args) -> int:
    from .fuse import FuseSource, fuse_stores, load_node_map, load_world_T_robot

    fused_dir = pathlib.Path(args.fused)
    node_map, merges = load_node_map(
        args.node_map or fused_dir / "backend" / "fusion_node_map.json"
    )
    sources = []
    for spec in args.source:
        robot, _, rest = spec.partition("=")
        store, _, session = rest.partition(":")
        if not store:
            print(
                f"bad --source {spec!r}; expected ROBOT_ID=STORE[:SESSION]",
                file=sys.stderr,
            )
            return 1
        world_T_robot = None
        pose_path = fused_dir / "interfaces" / f"robot_{robot}" / "world_T_robot.txt"
        if pose_path.exists():
            world_T_robot = load_world_T_robot(pose_path)
        sources.append(
            FuseSource(pathlib.Path(store), int(robot), session or None, world_T_robot)
        )

    link = "copy" if args.copy else ("symlink" if args.symlink else "hardlink")
    try:
        stats = fuse_stores(
            args.out, sources, node_map, merges, link=link, overwrite=args.overwrite
        )
    except (FileExistsError, ValueError) as exc:
        print(f"fuse failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(stats, indent=1, default=str))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hydra-image-store", description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("info", help="summarize one or more stores")
    s.add_argument("store", nargs="+", help="store directory or store.sqlite")
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=_cmd_info)

    s = sub.add_parser(
        "sessions",
        help="list the Hydra sessions of a run directory "
        "(rotated sessions/<k> and the live store) with their scene graphs",
    )
    s.add_argument("run_dir", help="run directory (or its live store)")
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=_cmd_sessions)

    s = sub.add_parser("export", help="write browsable per-node folders")
    s.add_argument("store", nargs="+")
    s.add_argument(
        "--dsg",
        help="scene graph used to correct poses (json, or anything spark_dsg loads)",
    )
    s.add_argument("--out", required=True)
    s.add_argument(
        "--copy", action="store_true", help="copy files instead of hard/symlinking"
    )
    s.add_argument("--kept-only", action="store_true", help="only kept observations")
    s.set_defaults(func=_cmd_export)

    s = sub.add_parser("gc", help="delete pixels no kept observation needs")
    s.add_argument("store")
    s.add_argument(
        "--max-images-per-object",
        type=int,
        default=None,
        help="re-apply keep-N over each merged node set",
    )
    s.add_argument(
        "--orphans", action="store_true", help="also delete unreferenced files"
    )
    s.add_argument("--dry-run", action="store_true")
    s.set_defaults(func=_cmd_gc)

    s = sub.add_parser("import-legacy", help="convert a pre-store run directory")
    s.add_argument("--run-dir", required=True)
    s.add_argument("--dsg", help="default: <run-dir>/hydra/backend/dsg.json if present")
    s.add_argument("--out", required=True, help="new store directory")
    s.add_argument("--robot-id", type=int, default=0)
    s.add_argument(
        "--sam3-dir", action="append", help="SAM3 object dirs (default: sam3_objects)"
    )
    s.add_argument(
        "--write-dsg",
        help="also write a copy of the scene graph without pre-store "
        "SubKeyframe nodes and image_folder attributes (loadable by upstream spark_dsg)",
    )
    g = s.add_mutually_exclusive_group()
    g.add_argument(
        "--copy", action="store_true", help="copy images (default: hardlink)"
    )
    g.add_argument(
        "--symlink", action="store_true", help="symlink images (default: hardlink)"
    )
    s.set_defaults(func=_cmd_import_legacy)

    s = sub.add_parser(
        "pack",
        help="pack a run's image store into one compressed archive "
        "(see hydra_image_store.pack)",
    )
    s.add_argument(
        "run_dir", help="run directory (uses <run>/image_store) or store directory"
    )
    s.add_argument("out", help="archive path (.tar, .tar.zst) or - for stdout")
    s.add_argument(
        "--frames",
        choices=["keyframes", "all"],
        default="keyframes",
        help="keyframes (+ kept object observation frames) or every frame with images",
    )
    s.add_argument(
        "--compression", choices=["none", "lossless", "fast"], default="fast"
    )
    s.add_argument(
        "--zstd-level",
        type=int,
        default=None,
        help="wrap the tar in zstd (default 3 for .zst paths, else off)",
    )
    s.add_argument(
        "--no-masks", action="store_true", help="do not ship kept mask PNGs (RLE stays)"
    )
    s.add_argument("--quiet", action="store_true")
    s.set_defaults(func=_cmd_pack)

    s = sub.add_parser(
        "unpack", help="rebuild a packed store as <dest>/image_store and check it"
    )
    s.add_argument("archive", help="archive path or - for stdin (zstd is detected)")
    s.add_argument("dest", help="destination run directory")
    s.add_argument("--store-name", default="image_store")
    s.add_argument("--overwrite", action="store_true")
    s.add_argument("--no-check", action="store_true")
    s.add_argument(
        "--dsg",
        help="scene graph for the checks (default <dest>/hydra/backend/dsg.json)",
    )
    s.add_argument("--quiet", action="store_true")
    s.set_defaults(func=_cmd_unpack)

    s = sub.add_parser("check", help="structural consistency checks of a store")
    s.add_argument("store", help="run directory or store directory")
    s.add_argument(
        "--dsg",
        help="also check keyframe anchors / bound nodes against this scene graph",
    )
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=_cmd_check)

    s = sub.add_parser(
        "fuse",
        help="merge per-robot / per-session stores into one store keyed "
        "by a Hydra-Multi fused scene graph",
    )
    s.add_argument(
        "fused",
        help="Hydra-Multi output directory (backend/fusion_node_map.json, "
        "interfaces/robot_<id>/world_T_robot.txt)",
    )
    s.add_argument(
        "--source",
        action="append",
        required=True,
        help="ROBOT_ID=STORE[:SESSION]: a source store and its robot id in the fused graph",
    )
    s.add_argument("--out", required=True, help="fused store directory")
    s.add_argument("--node-map", help="default: <fused>/backend/fusion_node_map.json")
    s.add_argument("--overwrite", action="store_true")
    g = s.add_mutually_exclusive_group()
    g.add_argument(
        "--copy", action="store_true", help="copy images (default: hardlink)"
    )
    g.add_argument(
        "--symlink", action="store_true", help="symlink images (default: hardlink)"
    )
    s.set_defaults(func=_cmd_fuse)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
