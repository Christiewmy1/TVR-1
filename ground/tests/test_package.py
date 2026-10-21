"""Smoke test: the ground_station package imports from the ground/ folder."""

import ground_station


def test_package_has_version():
    assert ground_station.__version__
