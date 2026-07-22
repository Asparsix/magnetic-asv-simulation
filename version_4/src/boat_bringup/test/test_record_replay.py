"""Sanity checks for record/replay helpers."""

import importlib.util
from pathlib import Path

import pytest
import yaml


BRINGUP = Path(__file__).parents[1]


def load_module(name, relative):
    path = BRINGUP / relative
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_record_topics_cover_pose_and_anomaly():
    config = yaml.safe_load(
        (BRINGUP / 'config' / 'record_topics.yaml').read_text(encoding='utf-8')
    )
    topics = set(config['topics'])
    assert '/asv1/pose2d' in topics
    assert '/asv1/cmd_vel' in topics
    assert '/asv1/mag/anomaly' in topics


def test_replay_rejects_unknown_mode(tmp_path):
    module = load_module('boat_replay_launch', 'launch/replay.launch.py')
    bag = tmp_path / 'dummy_bag'
    bag.mkdir()
    with pytest.raises(RuntimeError, match='from_pose'):
        module._replay_cmd(str(bag), '5.0', 'not_a_mode')


def test_replay_from_pose_selects_motion_topics(tmp_path):
    module = load_module('boat_replay_launch', 'launch/replay.launch.py')
    bag = tmp_path / 'dummy_bag'
    bag.mkdir()
    cmd = module._replay_cmd(str(bag), '5.0', 'from_pose')
    assert cmd[:3] == ['ros2', 'bag', 'play']
    assert '--clock' in cmd
    assert cmd[cmd.index('--rate') + 1] == '5.0'
    assert '/asv1/pose2d' in cmd
    assert '/asv1/cmd_vel' in cmd
    assert '/asv1/mag/anomaly' not in cmd


def test_replay_from_anomaly_selects_anomaly_topic(tmp_path):
    module = load_module('boat_replay_launch', 'launch/replay.launch.py')
    bag = tmp_path / 'dummy_bag'
    bag.mkdir()
    cmd = module._replay_cmd(str(bag), '10.0', 'from_anomaly')
    assert '/asv1/mag/anomaly' in cmd
    assert '/asv1/pose2d' not in cmd
