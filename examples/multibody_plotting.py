"""Shared math for the multibody plot scripts (constrained dynamics,
prescribed motion, robotic arms): upstream ``RigidBodyKinematics.MRP2C`` and
``subMRP``, and the constraint violations ``psi_B1`` and ``sigma_B2B1`` the
constrained-dynamics scenarios plot, from the hub columns the Rust examples
write (``r_B1N_N_m_i``, ``sigma_B1N_i``, ...)."""

from __future__ import annotations

import numpy as np

from orbit_plotting import Table


def mrp_to_dcm(sigma: np.ndarray) -> np.ndarray:
    """Upstream ``RigidBodyKinematics.MRP2C``: ``[BN]`` of ``sigma_BN``."""
    sigma = np.asarray(sigma, dtype=float)
    s2 = sigma @ sigma
    tilde = np.array([[0.0, -sigma[2], sigma[1]],
                      [sigma[2], 0.0, -sigma[0]],
                      [-sigma[1], sigma[0], 0.0]])
    return np.eye(3) + (8.0 * tilde @ tilde - 4.0 * (1.0 - s2) * tilde) / (1.0 + s2) ** 2


def _mrp_to_ep(sigma: np.ndarray) -> np.ndarray:
    s2 = sigma @ sigma
    return np.concatenate(([(1.0 - s2) / (1.0 + s2)], 2.0 * sigma / (1.0 + s2)))


def _ep_to_mrp(ep: np.ndarray) -> np.ndarray:
    if ep[0] >= 0.0:
        return ep[1:] / (1.0 + ep[0])
    return -ep[1:] / (1.0 - ep[0])


def sub_mrp(sigma: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Upstream ``RigidBodyKinematics.subMRP``: the attitude ``sigma``
    relative to ``reference``, composed through quaternions and returned as
    the short MRP set."""
    sigma = np.asarray(sigma, dtype=float)
    reference = np.asarray(reference, dtype=float)
    if np.array_equal(sigma, reference):
        return np.zeros(3)
    first, second = _mrp_to_ep(sigma), _mrp_to_ep(reference)
    vector = second[0] * first[1:] - first[0] * second[1:] + np.cross(first[1:], second[1:])
    scalar = first[0] * second[0] + first[1:] @ second[1:]
    return _ep_to_mrp(np.concatenate(([scalar], vector)))


def violations(table: Table, r_p1b1: np.ndarray, r_p2b2: np.ndarray,
               r_p2p1_init: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Upstream ``process_data``: ``psi_B1`` [m] and ``sigma_B2B1`` from both
    hub poses."""
    r1, s1 = table.vectors("r_B1N_N_m"), table.vectors("sigma_B1N")
    r2, s2 = table.vectors("r_B2N_N_m"), table.vectors("sigma_B2N")
    psi = np.empty(r1.shape)
    sigma_b2b1 = np.empty(s1.shape)
    for i in range(r1.shape[0]):
        dcm_b1n = mrp_to_dcm(s1[i])
        dcm_nb2 = mrp_to_dcm(s2[i]).T
        psi[i] = (dcm_b1n @ r1[i] + r_p1b1 + r_p2p1_init
                  - (dcm_b1n @ r2[i] + dcm_b1n @ dcm_nb2 @ r_p2b2))
        sigma_b2b1[i] = sub_mrp(s2[i], s1[i])
    return psi, sigma_b2b1


def violation_angles_deg(sigma_b2b1: np.ndarray) -> np.ndarray:
    """Upstream's plotted ``4 arctan(sigma_B2B1) R2D`` [deg]."""
    return np.degrees(4.0 * np.arctan(sigma_b2b1))


def model_geometry(model: str, r_hat: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``r_P1B1_B1``, ``r_P2B2_B2`` and ``r_P2P1_B1Init`` of upstream's
    ``sc_model`` along ``r_hat``."""
    r_p1b1, r_p2b2, r_p2p1 = {"bskSat": (0.1, 0.1, 0.1),
                              "MEV1": (1.0, 0.5, 0.5),
                              "MEV2": (1.0, 0.5, 0.5)}[model]
    r_hat = np.asarray(r_hat, dtype=float) / np.linalg.norm(r_hat)
    return r_p1b1 * r_hat, -r_p2b2 * r_hat, r_p2p1 * r_hat
