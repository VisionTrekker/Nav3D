"""Fixed-reference KISS-ICP registration with localization quality gates."""

from dataclasses import dataclass
import math

from kiss_icp.mapping import VoxelHashMap
from kiss_icp.registration import Registration
from kiss_icp.voxelization import voxel_down_sample
import numpy as np
from scipy.spatial import cKDTree

from lio_localization.localization_composer import _inverse_se3


@dataclass(frozen=True)
class RegistrationResult:
    accepted: bool
    transform: np.ndarray
    fitness: float
    correspondence_count: int
    translation_jump: float
    rotation_jump_degrees: float
    reason: str


class FixedMapRegistration:
    """Register base-frame scans against one immutable global voxel map."""

    def __init__(
        self,
        map_points: np.ndarray,
        *,
        voxel_size: float,
        max_correspondence_distance: float,
        max_iterations: int,
        convergence_criterion: float,
        kernel: float,
        fitness_threshold: float,
        min_correspondences: int,
        max_translation_jump: float,
        max_rotation_jump_degrees: float,
        max_points_per_voxel: int = 20,
    ) -> None:
        points = np.ascontiguousarray(map_points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
            raise ValueError('fixed map must be a nonempty Nx3 array')
        if not np.all(np.isfinite(points)):
            raise ValueError('fixed map contains NaN or Inf')
        if voxel_size <= 0.0 or max_correspondence_distance <= 0.0:
            raise ValueError('voxel and correspondence distances must be positive')
        if max_iterations <= 0 or convergence_criterion <= 0.0 or kernel <= 0.0:
            raise ValueError('ICP iteration, convergence, and kernel values must be positive')
        if fitness_threshold <= 0.0 or min_correspondences <= 0:
            raise ValueError('ICP fitness and correspondence gates must be positive')
        if max_translation_jump <= 0.0 or max_rotation_jump_degrees <= 0.0:
            raise ValueError('ICP jump gates must be positive')

        self.voxel_size = float(voxel_size)
        self.max_correspondence_distance = float(max_correspondence_distance)
        self.fitness_threshold = float(fitness_threshold)
        self.min_correspondences = int(min_correspondences)
        self.max_translation_jump = float(max_translation_jump)
        self.max_rotation_jump_degrees = float(max_rotation_jump_degrees)
        self.kernel = float(kernel)

        # add_points() is intentionally called exactly once.  Registration
        # scans are never passed to update()/add_points(), keeping the global
        # reference map immutable for the node's lifetime.
        self._voxel_map = VoxelHashMap(
            voxel_size=self.voxel_size,
            max_distance=1.0e9,
            max_points_per_voxel=int(max_points_per_voxel),
        )
        self._voxel_map.add_points(points)
        self._reference_points = np.ascontiguousarray(
            self._voxel_map.point_cloud(), dtype=np.float64)
        if self._reference_points.shape[0] == 0:
            raise ValueError('fixed map became empty after voxelization')
        self._reference_tree = cKDTree(self._reference_points)
        self._registration = Registration(
            max_num_iterations=int(max_iterations),
            convergence_criterion=float(convergence_criterion),
            max_num_threads=0,
        )

    @property
    def reference_point_count(self) -> int:
        return int(self._reference_points.shape[0])

    def register(self, source_points: np.ndarray, initial_guess: np.ndarray) -> RegistrationResult:
        source = np.ascontiguousarray(source_points, dtype=np.float64)
        guess = np.asarray(initial_guess, dtype=np.float64)
        if source.ndim != 2 or source.shape[1] != 3:
            return self._failure(guess, 'source cloud is not an Nx3 array')
        source = source[np.all(np.isfinite(source), axis=1)]
        if source.shape[0] < self.min_correspondences:
            return self._failure(
                guess, f'source has only {source.shape[0]} finite points')
        if guess.shape != (4, 4) or not np.all(np.isfinite(guess)):
            return self._failure(np.eye(4), 'initial guess is not a finite 4x4 matrix')
        if not self._is_se3(guess):
            return self._failure(np.eye(4), 'initial guess is not a valid SE(3) matrix')

        source = np.ascontiguousarray(
            voxel_down_sample(source, self.voxel_size), dtype=np.float64)
        if source.shape[0] < self.min_correspondences:
            return self._failure(
                guess, f'source has only {source.shape[0]} points after voxelization')

        try:
            transform = self._registration.align_points_to_map(
                points=source,
                voxel_map=self._voxel_map,
                initial_guess=np.ascontiguousarray(guess),
                max_correspondance_distance=self.max_correspondence_distance,
                kernel=self.kernel,
            )
        except (RuntimeError, TypeError, ValueError) as exc:
            return self._failure(guess, f'KISS-ICP raised {type(exc).__name__}: {exc}')

        transform = np.asarray(transform, dtype=np.float64)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            return self._failure(guess, 'KISS-ICP returned a non-finite transform')
        if not self._is_se3(transform):
            return self._failure(guess, 'KISS-ICP returned a matrix outside SE(3)')

        transformed = (transform[:3, :3] @ source.T).T + transform[:3, 3]
        distances, _ = self._reference_tree.query(
            transformed,
            k=1,
            distance_upper_bound=self.max_correspondence_distance,
            workers=-1,
        )
        valid_distances = distances[np.isfinite(distances)]
        correspondence_count = int(valid_distances.size)
        fitness = (
            float(np.sqrt(np.mean(valid_distances * valid_distances)))
            if correspondence_count else math.inf)

        delta = _inverse_se3(guess) @ transform
        translation_jump = float(np.linalg.norm(delta[:3, 3]))
        cosine = float(np.clip((np.trace(delta[:3, :3]) - 1.0) * 0.5, -1.0, 1.0))
        rotation_jump_degrees = float(np.degrees(np.arccos(cosine)))

        if correspondence_count < self.min_correspondences:
            reason = (
                f'only {correspondence_count} correspondences; '
                f'need {self.min_correspondences}')
        elif fitness > self.fitness_threshold:
            reason = (
                f'fitness {fitness:.6f} exceeds threshold '
                f'{self.fitness_threshold:.6f}')
        elif translation_jump > self.max_translation_jump:
            reason = (
                f'translation jump {translation_jump:.6f} m exceeds '
                f'{self.max_translation_jump:.6f} m')
        elif rotation_jump_degrees > self.max_rotation_jump_degrees:
            reason = (
                f'rotation jump {rotation_jump_degrees:.6f} deg exceeds '
                f'{self.max_rotation_jump_degrees:.6f} deg')
        else:
            return RegistrationResult(
                True, transform, fitness, correspondence_count,
                translation_jump, rotation_jump_degrees, 'accepted')

        return RegistrationResult(
            False, transform, fitness, correspondence_count,
            translation_jump, rotation_jump_degrees, reason)

    @staticmethod
    def _failure(transform: np.ndarray, reason: str) -> RegistrationResult:
        return RegistrationResult(
            False, np.asarray(transform), math.inf, 0, math.inf, math.inf, reason)

    @staticmethod
    def _is_se3(transform: np.ndarray) -> bool:
        rotation = transform[:3, :3]
        return bool(
            np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6) and
            np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-5) and
            np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-5)
        )
