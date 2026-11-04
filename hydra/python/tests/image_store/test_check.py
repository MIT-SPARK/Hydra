from hydra_python.image_store import ImageStore, check_store
from hydra_python.image_store.cli import main


def test_check_store_passes_and_detects_missing_files(synthetic, capsys):
    report = check_store(synthetic["dir"])
    assert report["ok"], report
    assert main(["check", str(synthetic["dir"])]) == 0
    assert "ALL PASS" in capsys.readouterr().out
    with ImageStore.open(synthetic["dir"]) as store:
        store.frame(synthetic["kf0"]).rgb_path.unlink()
    report = check_store(synthetic["dir"])
    assert not report["ok"] and not report["checks"]["all referenced files exist"]
    assert main(["check", str(synthetic["dir"])]) == 1


def test_check_store_resolves_run_dir(synthetic, tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    synthetic["dir"].rename(run / "image_store")
    assert check_store(run)["store"] == str((run / "image_store").absolute())
