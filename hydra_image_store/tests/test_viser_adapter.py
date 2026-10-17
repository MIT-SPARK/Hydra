from conftest import AGENT0, OBJ_A, OBJ_B

from hydra_image_store import ImageStore
from hydra_image_store.viser_adapter import make_image_provider

FIELDS = {"rgb_path", "depth_path", "mask_path", "timestamp_ns", "meta"}


def test_provider_objects_and_agents(synthetic):
    store = ImageStore.open(synthetic["dir"])
    prov = make_image_provider(store)
    refs = prov(OBJ_A)
    assert len(refs) == 2 and all(set(r) == FIELDS for r in refs)
    assert [r["meta"]["score"] for r in refs] == [0.9, 0.5]
    assert refs[0]["mask_path"] and refs[0]["depth_path"]
    # kept_only drops the unkept observation of object B
    assert len(prov(OBJ_B)) == 1
    assert len(make_image_provider(store, kept_only=False)(OBJ_B)) == 2
    agent = prov(AGENT0)
    assert [r["meta"]["kind"] for r in agent] == ["keyframe", "subkeyframe"]
    assert prov(12345) == []
