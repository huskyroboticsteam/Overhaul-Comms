"""Run the ROS 2 docstring style checks."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257() -> None:
    """Check the package with ament_pep257."""
    assert main(argv=['.', 'test']) == 0
