"""Orbital-element differences for the formation-flying plot scripts.

``plot_scenario_formation_mean_oe_feedback.py`` and
``plot_scenario_formation_reconfig.py`` redraw upstream's deputy-minus-chief
element difference figures from the chief and deputy states in their CSV
files. This module holds the parts of upstream ``orbitalMotion`` those
figures use beyond ``orbit_plotting.rv2elem``: the anomaly conversions, the
Brouwer J2 mean/osculating map (``clMeanOscMap``, as
``basilisk_rs::utilities::orbital_motion::mean_osculating_map``) and the
classical-to-equinoctial conversion (``clElem2eqElem``). Elements are dicts
with upstream's keys ``a``, ``e``, ``i``, ``Omega``, ``omega`` and ``f``.
"""
from __future__ import annotations

import math

# astroConstants REQ_EARTH [m] and J2_EARTH, as the scenarios use them.
REQ_EARTH = 6378.1366e3
J2_EARTH = 1082.616e-6


def f2E(f: float, e: float) -> float:
    """Upstream ``orbitalMotion.f2E`` (elliptic orbits)."""
    return 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(f / 2.0),
                            math.sqrt(1.0 + e) * math.cos(f / 2.0))


def E2f(E: float, e: float) -> float:
    """Upstream ``orbitalMotion.E2f`` (elliptic orbits)."""
    return 2.0 * math.atan2(math.sqrt(1.0 + e) * math.sin(E / 2.0),
                            math.sqrt(1.0 - e) * math.cos(E / 2.0))


def E2M(E: float, e: float) -> float:
    """Upstream ``orbitalMotion.E2M``."""
    return E - e * math.sin(E)


def M2E(M: float, e: float) -> float:
    """Upstream ``orbitalMotion.M2E``: Newton iteration on Kepler's equation."""
    E = M
    for _ in range(200):
        step = (E - e * math.sin(E) - M) / (1.0 - e * math.cos(E))
        E -= step
        if abs(step) < 1e-13:
            break
    return E


def mean_anomaly(oe: dict[str, float]) -> float:
    return E2M(f2E(oe["f"], oe["e"]), oe["e"])


def mean_osc_map(oe: dict[str, float], sign: float, req: float = REQ_EARTH,
                 j2: float = J2_EARTH) -> dict[str, float]:
    """Upstream ``orbitalMotion.clMeanOscMap(req, J2, oe, oep, sign)``:
    Brouwer's first-order J2 map, ``sign = -1`` from osculating to mean
    elements and ``+1`` back (Schaub & Junkins, Appendix F)."""
    a, e, i, raan, w, f = (oe[key] for key in ("a", "e", "i", "Omega", "omega", "f"))
    m = mean_anomaly(oe)

    gamma2 = sign * j2 / 2.0 * (req / a) ** 2
    eta = math.sqrt(1.0 - e * e)
    gamma2p = gamma2 / eta**4
    a_r = (1.0 + e * math.cos(f)) / eta**2

    ci = math.cos(i)
    ci2, ci4, ci6 = ci**2, ci**4, ci**6
    si2 = 1.0 - ci2
    critical = 1.0 - 5.0 * ci2
    cf, sf = math.cos(f), math.sin(f)
    sin_2w, cos_2w = math.sin(2.0 * w), math.cos(2.0 * w)
    w2_f, w2_2f, w2_3f = 2.0 * w + f, 2.0 * w + 2.0 * f, 2.0 * w + 3.0 * f
    kozai = 1.0 - 11.0 * ci2 - 40.0 * ci4 / critical
    node_bracket = 11.0 + 80.0 * ci2 / critical + 200.0 * ci4 / critical**2
    center = f - m + e * sf
    short_period = (6.0 * center - 3.0 * math.sin(w2_2f) - 3.0 * e * math.sin(w2_f)
                    - e * math.sin(w2_3f))

    ap = a + a * gamma2 * ((3.0 * ci2 - 1.0) * (a_r**3 - 1.0 / eta**3)
                           + 3.0 * si2 * a_r**3 * math.cos(w2_2f))
    de1 = gamma2p / 8.0 * e * eta**2 * kozai * cos_2w
    de = de1 + eta**2 / 2.0 * (
        gamma2 * ((3.0 * ci2 - 1.0) / eta**6
                  * (e * eta + e / (1.0 + eta) + 3.0 * cf + 3.0 * e * cf**2 + e**2 * cf**3)
                  + 3.0 * si2 / eta**6
                  * (e + 3.0 * cf + 3.0 * e * cf**2 + e**2 * cf**3) * math.cos(w2_2f))
        - gamma2p * si2 * (3.0 * math.cos(w2_f) + math.cos(w2_3f)))
    di = (-e * de1 / eta**2 / math.tan(i)
          + gamma2p / 2.0 * ci * math.sqrt(si2)
          * (3.0 * math.cos(w2_2f) + 3.0 * e * math.cos(w2_f) + e * math.cos(w2_3f)))
    mean_lon_p = (m + w + raan + gamma2p / 8.0 * eta**3 * kozai * sin_2w
                  - gamma2p / 16.0 * (2.0 + e**2 - 11.0 * (2.0 + 3.0 * e**2) * ci2
                                      - 40.0 * (2.0 + 5.0 * e**2) * ci4 / critical
                                      - 400.0 * e**2 * ci6 / critical**2) * sin_2w
                  + gamma2p / 4.0 * (-6.0 * critical * center
                                     + (3.0 - 5.0 * ci2) * (3.0 * math.sin(w2_2f)
                                                            + 3.0 * e * math.sin(w2_f)
                                                            + e * math.sin(w2_3f)))
                  - gamma2p / 8.0 * e**2 * ci * node_bracket * sin_2w
                  - gamma2p / 2.0 * ci * short_period)
    are2 = (a_r * eta) ** 2
    e_dm = (gamma2p / 8.0 * e * eta**3 * kozai * sin_2w
            - gamma2p / 4.0 * eta**3 * (2.0 * (3.0 * ci2 - 1.0) * (are2 + a_r + 1.0) * sf
                                       + 3.0 * si2 * ((-are2 - a_r + 1.0) * math.sin(w2_f)
                                                      + (are2 + a_r + 1.0 / 3.0)
                                                      * math.sin(w2_3f))))
    d_raan = (-gamma2p / 8.0 * e**2 * ci * node_bracket * sin_2w
              - gamma2p / 2.0 * ci * short_period)

    d1 = (e + de) * math.sin(m) + e_dm * math.cos(m)
    d2 = (e + de) * math.cos(m) - e_dm * math.sin(m)
    mp = math.atan2(d1, d2)
    ep = math.sqrt(d1 * d1 + d2 * d2)
    sin_half_i, cos_half_i = math.sin(i / 2.0), math.cos(i / 2.0)
    node_sin = sin_half_i + cos_half_i * di / 2.0
    d3 = node_sin * math.sin(raan) + sin_half_i * d_raan * math.cos(raan)
    d4 = node_sin * math.cos(raan) - sin_half_i * d_raan * math.sin(raan)
    raan_p = math.atan2(d3, d4)
    ip = 2.0 * math.asin(max(-1.0, min(1.0, math.sqrt(d3 * d3 + d4 * d4))))
    wp = mean_lon_p - mp - raan_p
    return {"a": ap, "e": ep, "i": ip, "Omega": raan_p, "omega": wp,
            "f": E2f(M2E(mp, ep), ep)}


def classic_to_equinoctial(oe: dict[str, float]) -> dict[str, float]:
    """Upstream ``orbitalMotion.clElem2eqElem``: ``a``, ``P1``, ``P2``,
    ``Q1``, ``Q2`` and the mean longitude ``l``."""
    longitude_of_periapsis = oe["Omega"] + oe["omega"]
    tan_half_i = math.tan(oe["i"] / 2.0)
    return {
        "a": oe["a"],
        "P1": oe["e"] * math.sin(longitude_of_periapsis),
        "P2": oe["e"] * math.cos(longitude_of_periapsis),
        "Q1": tan_half_i * math.sin(oe["Omega"]),
        "Q2": tan_half_i * math.cos(oe["Omega"]),
        "l": longitude_of_periapsis + mean_anomaly(oe),
    }


def wrap_pi(angle: float) -> float:
    """Upstream's ``while angle > pi: angle -= 2 pi`` (and below ``-pi``)."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def classic_difference(chief: dict[str, float], deputy: dict[str, float]) -> list[float]:
    """Upstream's ``oed``: ``(a2 - a)/a``, ``de``, ``di``, ``dOmega``,
    ``domega`` and ``dM``, the angles wrapped to ``[-pi, pi]``."""
    difference = [
        (deputy["a"] - chief["a"]) / chief["a"],
        deputy["e"] - chief["e"],
        deputy["i"] - chief["i"],
        deputy["Omega"] - chief["Omega"],
        deputy["omega"] - chief["omega"],
        mean_anomaly(deputy) - mean_anomaly(chief),
    ]
    return difference[:3] + [wrap_pi(angle) for angle in difference[3:]]


def equinoctial_difference(chief: dict[str, float], deputy: dict[str, float]) -> list[float]:
    """Upstream's ``oed_eq``: ``(a2 - a)/a``, ``dP1``, ``dP2``, ``dQ1``,
    ``dQ2`` and ``dl``, the mean longitude wrapped to ``[-pi, pi]``."""
    chief_eq, deputy_eq = classic_to_equinoctial(chief), classic_to_equinoctial(deputy)
    return [
        (deputy_eq["a"] - chief_eq["a"]) / chief_eq["a"],
        deputy_eq["P1"] - chief_eq["P1"],
        deputy_eq["P2"] - chief_eq["P2"],
        deputy_eq["Q1"] - chief_eq["Q1"],
        deputy_eq["Q2"] - chief_eq["Q2"],
        wrap_pi(deputy_eq["l"] - chief_eq["l"]),
    ]
