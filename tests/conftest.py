from pathlib import Path

import pytest

from collision_checker.model import Robot
from collision_checker.state import Session

FIXTURE = Path(__file__).parent / "fixtures" / "sample.urdf"


@pytest.fixture
def robot():
    return Robot(FIXTURE)


@pytest.fixture
def session(robot):
    session = Session(robot, "test")
    session.settings["samples"] = 128
    return session
