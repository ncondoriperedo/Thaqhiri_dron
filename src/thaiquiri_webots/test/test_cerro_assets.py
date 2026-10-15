"""Verify the exported mine is portable and preserves physical scale."""

import json
from pathlib import Path

import pytest


def test_mine_assets_and_collision_bounds():
    root = Path(__file__).resolve().parents[1] / 'webots_world/protos/cerro_rico'
    manifest = json.loads((root / 'manifest.json').read_text())
    assert manifest['metres_per_unit'] == 1
    assert manifest['collision_faces'] > 70000
    assert manifest['spawn_marker'] == pytest.approx([0, 3, 1.2])
    assert (root / 'surface_reference.npz').is_file()
    for line in (root / 'visual.mtl').read_text().splitlines():
        if line.startswith('map_Kd '):
            texture = root / line.split(maxsplit=1)[1]
            assert texture.is_file()
            assert texture.resolve().is_relative_to(root.resolve())
    vertices = [list(map(float, line.split()[1:4])) for line in
                (root / 'collision.obj').read_text().splitlines() if line.startswith('v ')]
    for axis in range(3):
        bounds = [min(v[axis] for v in vertices), max(v[axis] for v in vertices)]
        assert bounds == pytest.approx(manifest['bounds'][axis], abs=1e-4)


def test_onboard_light_is_switchable_in_the_physical_model():
    proto = (Path(__file__).resolve().parents[1] /
             'webots_world/protos/thaiquiri.proto').read_text()
    assert 'field SFBool      headlightOn  TRUE' in proto
    assert 'SpotLight {' in proto
    assert 'on IS headlightOn' in proto
