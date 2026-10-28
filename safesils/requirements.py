"""Loader for the safety requirement catalogue (requirements/aeb_safety.yaml)."""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import yaml

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "requirements" / "aeb_safety.yaml"


@lru_cache(maxsize=4)
def load(path: str | Path = DEFAULT_PATH) -> dict:
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    index: dict[str, dict] = {}
    for key in ("hazards", "safety_goals", "functional_safety_requirements", "technical_safety_requirements"):
        for req in doc.get(key, []):
            index[req["id"]] = req
    doc["index"] = index
    return doc


def get(req_id: str) -> dict:
    return load()["index"][req_id]


def params(req_id: str) -> dict:
    return get(req_id).get("params", {})


def parents(req_id: str) -> list[str]:
    """Chain from a requirement up to its safety goal, e.g. TSR-02 -> FSR-02 -> SG-01."""
    chain, cur = [], get(req_id)
    while "parent" in cur:
        chain.append(cur["parent"])
        cur = get(cur["parent"])
    return chain
