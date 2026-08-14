"""Checks that fast mode changes timing, not physics integration fidelity."""

import importlib.util
from pathlib import Path

import pytest


LAUNCH_FILE = (
    Path(__file__).parents[1] / 'launch' / 'sim.launch.py'
)


def load_launch_module():
    spec = importlib.util.spec_from_file_location('boat_sim_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_fast_world_changes_only_timing_settings():
    module = load_launch_module()
    original = (
        '<physics>'
        '<max_step_size>0.001</max_step_size>'
        '<real_time_factor>1.0</real_time_factor>'
        '</physics>'
    )
    fast = module._fast_world_sdf(original, 2.0, 0.002)

    assert '<real_time_factor>2</real_time_factor>' in fast
    assert '<max_step_size>0.002</max_step_size>' in fast
    restored = fast.replace(
        '<real_time_factor>2</real_time_factor>',
        '<real_time_factor>1.0</real_time_factor>',
    ).replace(
        '<max_step_size>0.002</max_step_size>',
        '<max_step_size>0.001</max_step_size>',
    )
    assert restored == original


@pytest.mark.parametrize('factor', [1.0, 0.0, -2.0, 10.1])
def test_fast_factor_rejects_unsafe_values(factor):
    module = load_launch_module()
    with pytest.raises(ValueError):
        module._fast_world_sdf(
            (
                '<max_step_size>0.001</max_step_size>'
                '<real_time_factor>1.0</real_time_factor>'
            ),
            factor,
            0.002,
        )


@pytest.mark.parametrize('step', [0.0009, 0.0041])
def test_fast_step_rejects_unsafe_values(step):
    module = load_launch_module()
    with pytest.raises(ValueError):
        module._fast_world_sdf(
            (
                '<max_step_size>0.001</max_step_size>'
                '<real_time_factor>1.0</real_time_factor>'
            ),
            2.0,
            step,
        )
