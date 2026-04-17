#!/usr/bin/env python3

"""Pure-Python particle-filter helpers used by the ROS runtime node."""

import math
import random
from dataclasses import dataclass

from pf_math import Pose2D, wrap_angle


@dataclass
class Particle:
    pose: Pose2D
    weight: float


def initialize_global_particles(field_map, count, rng=None):
    rng = rng if rng is not None else random.Random()
    if count <= 0:
        raise ValueError("Particle count must be positive.")

    weight = 1.0 / float(count)
    particles = []
    for _ in range(count):
        x, y, theta = field_map.random_free_pose(rng)
        particles.append(Particle(Pose2D(x, y, theta), weight))
    return particles


def normalize_log_weights(log_weights):
    if not log_weights:
        return []

    max_log_weight = max(log_weights)
    if not math.isfinite(max_log_weight):
        return [1.0 / len(log_weights)] * len(log_weights)

    weights = [math.exp(value - max_log_weight) for value in log_weights]
    total = sum(weights)
    if total <= 0.0 or not math.isfinite(total):
        return [1.0 / len(log_weights)] * len(log_weights)
    return [weight / total for weight in weights]


def normalize_particle_log_weights(particles, log_likelihoods, min_weight=1.0e-300):
    if len(particles) != len(log_likelihoods):
        raise ValueError("Particle and likelihood counts must match.")

    combined_log_weights = []
    for particle, log_likelihood in zip(particles, log_likelihoods):
        prior_log_weight = math.log(max(float(particle.weight), min_weight))
        combined_log_weights.append(prior_log_weight + log_likelihood)
    return normalize_log_weights(combined_log_weights)


def effective_sample_size(weights):
    denominator = sum(weight * weight for weight in weights)
    if denominator <= 0.0:
        return 0.0
    return 1.0 / denominator


def estimate_pose(particles):
    if not particles:
        raise ValueError("Cannot estimate pose from an empty particle set.")

    total_weight = sum(particle.weight for particle in particles)
    if total_weight <= 0.0:
        total_weight = float(len(particles))
        weights = [1.0 / total_weight] * len(particles)
    else:
        weights = [particle.weight / total_weight for particle in particles]

    x = sum(weight * particle.pose.x for weight, particle in zip(weights, particles))
    y = sum(weight * particle.pose.y for weight, particle in zip(weights, particles))
    sin_theta = sum(
        weight * math.sin(particle.pose.theta)
        for weight, particle in zip(weights, particles)
    )
    cos_theta = sum(
        weight * math.cos(particle.pose.theta)
        for weight, particle in zip(weights, particles)
    )
    theta = math.atan2(sin_theta, cos_theta)
    return Pose2D(x, y, theta)


def low_variance_resample(particles, rng=None):
    rng = rng if rng is not None else random.Random()
    count = len(particles)
    if count == 0:
        return []

    weights = [particle.weight for particle in particles]
    total = sum(weights)
    if total <= 0.0:
        weights = [1.0 / count] * count
    else:
        weights = [weight / total for weight in weights]

    step = 1.0 / count
    offset = rng.uniform(0.0, step)
    cumulative = weights[0]
    index = 0
    resampled = []

    for sample_index in range(count):
        threshold = offset + sample_index * step
        while threshold > cumulative and index < count - 1:
            index += 1
            cumulative += weights[index]
        resampled.append(Particle(particles[index].pose, step))

    return resampled


def yaw_error(estimated_theta, true_theta):
    return abs(wrap_angle(estimated_theta - true_theta))


def position_error(estimated_pose, true_pose):
    return math.hypot(estimated_pose.x - true_pose.x, estimated_pose.y - true_pose.y)
