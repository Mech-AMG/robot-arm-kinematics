import pytest

from project import (
    forward_kinematics,
    forward_kinematics_3d,
    interpolate_joints,
    inverse_kinematics,
    inverse_kinematics_3d,
)


def test_forward_kinematics():
    assert forward_kinematics(10, 10, 0, 0) == pytest.approx((20, 0))
    assert forward_kinematics(10, 10, 90, 0) == pytest.approx((0, 20))


def test_inverse_kinematics_reconstructs_target():
    target = (12, 8)
    solutions = inverse_kinematics(10, 8, *target)
    for angles in solutions:
        assert forward_kinematics(10, 8, *angles) == pytest.approx(target)


def test_inverse_kinematics_rejects_unreachable_target():
    with pytest.raises(ValueError):
        inverse_kinematics(5, 5, 20, 20)


def test_forward_kinematics_3d():
    assert forward_kinematics_3d(10, 10, 0, 0, 0) == pytest.approx(
        (20, 0, 0)
    )
    assert forward_kinematics_3d(10, 10, 90, 0, 0) == pytest.approx(
        (0, 20, 0)
    )


def test_inverse_kinematics_3d_reconstructs_target():
    target = (6, 8, 8)
    for solution in inverse_kinematics_3d(10, 8, *target):
        assert forward_kinematics_3d(10, 8, *solution) == pytest.approx(target)


def test_interpolate_joints():
    path = interpolate_joints((0, 10), (90, 30), 4)
    expected = [(0, 10), (30, 50 / 3), (60, 70 / 3), (90, 30)]
    for actual, target in zip(path, expected):
        assert actual[0] == pytest.approx(target[0])
        assert actual[1] == pytest.approx(target[1])
    assert len(interpolate_joints((0, 0, 0), (30, 60, 90), 2)) == 2


def test_invalid_trajectory():
    with pytest.raises(ValueError):
        interpolate_joints((0, 0), (1, 1), 1)
