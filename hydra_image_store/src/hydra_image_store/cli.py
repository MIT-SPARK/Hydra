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
        print(f"  schema_version {s['meta'].get('schema_version')}  robots {s['robots']}  sensors {s['sensors']}")
        frames = ", ".join(f"{k} {v}" for k, v in s["frames"].items()) or "none"
        print(f"  frames: {frames}  (with rgb {s['frames_with_rgb']}, unanchored {s['frames_unanchored']})")
        print(f"  observations {s['observations']} (kept {s['observations_kept']}) over {s['tracks']} tracks")
        print(f"  track bindings {s['track_bindings']} -> {s['bound_nodes']} nodes, node aliases {s['node_aliases']}")
        extra = {k: v for k, v in s["meta"].items() if k != "schema_version"}
        if extra:
            print(f"  meta {extra}")
    return 0


def _cmd_export(args) -> int:
    from .export import export_store
    from .poses import load_dsg

    pose_source = load_dsg(args.dsg) if args.dsg else None
    with ImageStore.open(*args.store) as store:
        stats = export_store(store, args.out, pose_source, copy=args.copy, kept_only=args.kept_only)
    print(json.dumps(stats, indent=1))
    return 0


def _cmd_gc(args) -> int:
    from .gc import gc_store

    with ImageStoreWriter(args.store) as writer:
        stats = gc_store(writer, args.max_images_per_object, orphans=args.orphans, dry_run=args.dry_run)
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
            raise SystemExit("--write-dsg needs --dsg (no scene graph found in the run dir)")
        stats["stripped_dsg"] = strip_legacy_dsg(src, args.write_dsg)
    print(json.dumps(stats, indent=1))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hydra-image-store", description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("info", help="summarize one or more stores")
    s.add_argument("store", nargs="+", help="store directory or store.sqlite")
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=_cmd_info)

    s = sub.add_parser("export", help="write browsable per-node folders")
    s.add_argument("store", nargs="+")
    s.add_argument("--dsg", help="scene graph used to correct poses (json, or anything spark_dsg loads)")
    s.add_argument("--out", required=True)
    s.add_argument("--copy", action="store_true", help="copy files instead of hard/symlinking")
    s.add_argument("--kept-only", action="store_true", help="only kept observations")
    s.set_defaults(func=_cmd_export)

    s = sub.add_parser("gc", help="delete pixels no kept observation needs")
    s.add_argument("store")
    s.add_argument("--max-images-per-object", type=int, default=None,
                   help="re-apply keep-N over each merged node set")
    s.add_argument("--orphans", action="store_true", help="also delete unreferenced files")
    s.add_argument("--dry-run", action="store_true")
    s.set_defaults(func=_cmd_gc)

    s = sub.add_parser("import-legacy", help="convert a pre-store run directory")
    s.add_argument("--run-dir", required=True)
    s.add_argument("--dsg", help="default: <run-dir>/hydra/backend/dsg.json if present")
    s.add_argument("--out", required=True, help="new store directory")
    s.add_argument("--robot-id", type=int, default=0)
    s.add_argument("--sam3-dir", action="append", help="SAM3 object dirs (default: sam3_objects)")
    s.add_argument("--write-dsg", help="also write a copy of the scene graph without pre-store "
                   "SubKeyframe nodes and image_folder attributes (loadable by upstream spark_dsg)")
    g = s.add_mutually_exclusive_group()
    g.add_argument("--copy", action="store_true", help="copy images (default: hardlink)")
    g.add_argument("--symlink", action="store_true", help="symlink images (default: hardlink)")
    s.set_defaults(func=_cmd_import_legacy)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
