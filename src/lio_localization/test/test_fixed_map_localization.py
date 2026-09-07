from pathlib import Path

import numpy as np

from lio_localization.fixed_map_registration import FixedMapRegistration
from lio_localization.localization_composer import _inverse_se3
from lio_localization.pcd_io import load_pcd_xyz


def _fixed_points() -> np.ndarray:
    rng = np.random.default_rng(17)
    floor = np.column_stack((
        rng.uniform(-3.0, 4.0, 1200),
        rng.uniform(-2.0, 3.0, 1200),
        np.zeros(1200),
    ))
    wall_x = np.column_stack((
        np.full(900, 3.5),
        rng.uniform(-2.0, 3.0, 900),
        rng.uniform(0.0, 2.5, 900),
    ))
    wall_y = np.column_stack((
        rng.uniform(-3.0, 4.0, 800),
        np.full(800, -2.0),
        rng.uniform(0.0, 2.5, 800),
    ))
    asymmetric_cluster = rng.normal(
        [0.7, 1.1, 1.4], [0.35, 0.2, 0.25], (500, 3))
    return np.vstack((floor, wall_x, wall_y, asymmetric_cluster))


def _transform(translation, yaw_degrees=0.0) -> np.ndarray:
    yaw = np.deg2rad(yaw_degrees)
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    result[:3, 3] = translation
    return result


def _registration(points) -> FixedMapRegistration:
    return FixedMapRegistration(
        points,
        voxel_size=0.15,
        max_correspondence_distance=1.0,
        max_iterations=100,
        convergence_criterion=1.0e-5,
        kernel=0.5,
        fitness_threshold=0.08,
        min_correspondences=100,
        max_translation_jump=0.7,
        max_rotation_jump_degrees=10.0,
    )


def _pose_error(truth: np.ndarray, estimate: np.ndarray):
    error = _inverse_se3(truth) @ estimate
    translation = np.linalg.norm(error[:3, 3])
    cosine = np.clip((np.trace(error[:3, :3]) - 1.0) * 0.5, -1.0, 1.0)
    rotation_degrees = np.degrees(np.arccos(cosine))
    return translation, rotation_degrees


def test_icp_recovers_identity_from_small_translation_error():
    fixed = _fixed_points()
    initial = _transform([0.18, -0.12, 0.08])
    registration = _registration(fixed)
    reference_count = registration.reference_point_count
    result = registration.register(fixed, initial)

    translation_error, rotation_error = _pose_error(np.eye(4), result.transform)
    assert result.accepted, result.reason
    assert registration.reference_point_count == reference_count
    assert translation_error < 0.02
    assert rotation_error < 0.2


def test_icp_recovers_known_translation_and_yaw():
    fixed = _fixed_points()
    truth = _transform([1.2, -0.7, 0.25], yaw_degrees=18.0)
    truth_inverse = _inverse_se3(truth)
    source = (truth_inverse[:3, :3] @ fixed.T).T + truth_inverse[:3, 3]
    initial = truth @ _transform([0.16, -0.10, 0.07], yaw_degrees=2.5)
    result = _registration(fixed).register(source, initial)

    translation_error, rotation_error = _pose_error(truth, result.transform)
    assert result.accepted, result.reason
    assert translation_error < 0.02
    assert rotation_error < 0.2


def test_composer_correction_chain_is_full_se3():
    odom_to_base = _transform([1.3, -0.4, 0.2], yaw_degrees=11.0)
    corrected_map_to_base = _transform([7.2, 3.1, 1.0], yaw_degrees=-23.0)

    map_to_odom = corrected_map_to_base @ _inverse_se3(odom_to_base)
    recomposed = map_to_odom @ odom_to_base

    assert np.allclose(recomposed, corrected_map_to_base, atol=1.0e-12)


def test_quality_gate_rejects_cloud_without_correspondences():
    fixed = _fixed_points()
    source = fixed + np.array([100.0, 100.0, 100.0])
    result = _registration(fixed).register(source, np.eye(4))

    assert not result.accepted
    assert result.correspondence_count == 0


def test_ascii_pcd_loader_filters_nonfinite_points(tmp_path: Path):
    path = tmp_path / 'map.pcd'
    path.write_text(
        'VERSION .7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        'WIDTH 3\n'
        'HEIGHT 1\n'
        'POINTS 3\n'
        'DATA ascii\n'
        '0 0 0\n'
        '1 2 3\n'
        'nan 5 6\n',
        encoding='ascii',
    )

    points = load_pcd_xyz(path)
    assert np.allclose(points, [[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]])
