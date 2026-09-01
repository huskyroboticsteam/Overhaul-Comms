"""Tests for mc_bridge package metadata."""

from pathlib import Path
import runpy
from unittest.mock import patch


def test_data_file_sources_are_relative_and_exist() -> None:
    """Package data must satisfy colcon and resolve from the package root."""
    package_root = Path(__file__).resolve().parents[1]

    with patch('setuptools.setup') as setup:
        runpy.run_path(str(package_root / 'setup.py'))

    data_files = setup.call_args.kwargs['data_files']
    sources = [source for _, group in data_files for source in group]

    assert sources
    assert all(not Path(source).is_absolute() for source in sources)
    assert all((package_root / source).is_file() for source in sources)
