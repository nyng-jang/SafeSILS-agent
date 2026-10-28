"""Back-to-back tests of the MATLAB controller code and of the FMU adapter."""
from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from safesils import generator
from safesils.b2b import closed_loop, open_loop

BASELINE = {s.tc_id: s for s in generator.baseline()}
B2B_SET = ["TC-SG03-001", "TC-TSR01-001", "TC-TSR03-002", "TC-TSR04-001", "TC-TSR05-001", "TC-TSR07-001"]


@pytest.mark.skipif(not (shutil.which("octave") or shutil.which("matlab")), reason="needs GNU Octave or MATLAB")
@pytest.mark.parametrize("tc_id", B2B_SET)
def test_matlab_code_matches_python(tc_id: str) -> None:
    assert open_loop(BASELINE[tc_id]) == []


@pytest.fixture(scope="module")
def stub_fmu(tmp_path_factory) -> Path:
    pytest.importorskip("pythonfmu")
    pytest.importorskip("fmpy")
    out = tmp_path_factory.mktemp("fmu")
    stub = Path(__file__).parent / "fmu_stub" / "AEB_Controller.py"
    subprocess.run([sys.executable, "-m", "pythonfmu", "build", "-f", str(stub), "-d", str(out)],
                   check=True, cwd=stub.parent, capture_output=True)
    return out / "AEB_Controller.fmu"


@pytest.mark.parametrize("tc_id", ["TC-SG03-001", "TC-TSR02-001", "TC-TSR08-001"])
def test_fmu_adapter_closed_loop(stub_fmu: Path, tc_id: str) -> None:
    from safesils.external import FMUController

    ctrl = FMUController(str(stub_fmu))
    try:
        assert closed_loop(BASELINE[tc_id], ctrl) == []
    finally:
        ctrl.close()


VECTORS = Path(__file__).resolve().parent.parent / "matlab" / "b2b_vectors"


def test_committed_b2b_vectors_are_current(tmp_path: Path) -> None:
    """matlab/b2b_vectors must match the Python reference; regenerate with
    `safesils b2b scenarios/baseline --export matlab/b2b_vectors` after controller changes."""
    from safesils.b2b import export_vectors
    from safesils.scenario import Scenario

    scenarios = sorted((VECTORS.parent.parent / "scenarios" / "baseline").glob("*.json"))
    assert len(list(VECTORS.glob("*.in.csv"))) == len(scenarios)
    for f in scenarios:
        for new in export_vectors(Scenario.load(f), tmp_path):
            assert new.read_text() == (VECTORS / new.name).read_text(), f"{new.name} is stale"


@pytest.mark.skipif(not shutil.which("octave"), reason="needs GNU Octave")
def test_b2b_simulink_script_without_model() -> None:
    cmd = f"addpath('{VECTORS.parent.as_posix()}'); exit(~b2b_simulink('', false));"
    res = subprocess.run(["octave", "--no-gui", "--quiet", "--eval", cmd], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
