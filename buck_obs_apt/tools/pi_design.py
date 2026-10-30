#!/usr/bin/env python3
"""Discrete-time PI design for the cascaded average-current-mode loop.

Textbook rule: each PI is chosen so that its loop gain crosses 0 dB at the
target crossover with 45 deg phase margin. With two free parameters (Kp, Ki)
and two conditions (|L| = 1, angle L = -135 deg at f_c) the gains follow
directly from the frequency response of the sampled plant.

Plant (averaged, per switching cycle T = 1/600 kHz):
    L di/dt = D Vin - v - r i,   C dv/dt = i - v/R
Input D is the PHYSICAL on-time fraction (dead time is off-time; the firmware
adds it to CMP1), held for one cycle (ZOH). Feedback signals:
    current loop : cycle-average inductor current of cycle k  (Gai)
    voltage loop : v_out sampled at the start of cycle k       (Gv)
Firmware schedule (control update during cycle n): v[n] and the average over cycle n-1
are read, i_ref[n] = Cv(Vref - v[n]), D[n+1] = Ci(i_ref[n] - iavg[n-1]).
Hence the loop gains
    Li(z) = Ci(z) z^-2 Gai(z)
    Lv(z) = Cv(z) Gv(z) z^-1 Ci(z) / (1 + Ci(z) z^-2 Gai(z))
with the PI in the firmware's form C(z) = Kp + Ki T z/(z-1)
(integ += Ki T e; u = Kp e + integ).

Design point: no load (worst case for the voltage-loop phase margin; heavier
loads add phase). Run:  python3 tools/pi_design.py [f_ci_Hz f_cv_Hz]
"""
import sys
import numpy as np
from scipy.linalg import expm

T, L, C, R_L, VIN = 1 / 600e3, 330e-9, 270e-6, 0.114, 6.0
R_DESIGN = 1e9  # no load


def plant(R):
    A = np.array([[-R_L / L, -1 / L], [1 / C, -1 / (C * R)]])
    B = np.array([[VIN / L], [0.0]])
    n = 2
    M = np.zeros((3 * n, 3 * n))
    M[:n, :n] = A
    M[:n, n:2 * n] = np.eye(n)
    M[n:2 * n, 2 * n:] = np.eye(n)
    E = expm(M * T)
    Ad, P1, P2 = E[:n, :n], E[:n, n:2 * n], E[:n, 2 * n:]
    return Ad, P1 @ B, P1[0:1, :] / T, (P2 @ B)[0:1, :] / T


def resp(R, f):
    Ad, Bd, Cav, Dav = plant(R)
    z = np.exp(1j * 2 * np.pi * f * T)
    X = np.linalg.solve(z * np.eye(2) - Ad, Bd)
    return z, (Cav @ X + Dav)[0, 0], X[1, 0]


def pi_c(kp, ki, z):
    return kp + ki * T * z / (z - 1)


def solve_pi(P, z, pm_deg=45.0):
    c = np.exp(1j * np.deg2rad(-180 + pm_deg)) / P
    w = z / (z - 1)
    ki = c.imag / (T * w.imag)
    return c.real - ki * T * w.real, ki


def li(g, R, f):
    z, gai, _ = resp(R, f)
    return pi_c(g[0], g[1], z) * z ** -2 * gai


def lv(gi, gv, R, f):
    z, gai, gv_ = resp(R, f)
    ci = pi_c(gi[0], gi[1], z)
    return pi_c(gv[0], gv[1], z) * gv_ / z * ci / (1 + ci * z ** -2 * gai)


def margins(fun):
    fs = np.linspace(50, 299.9e3, 60000)
    Ls = np.array([fun(f) for f in fs])
    mag, ph = np.abs(Ls), np.rad2deg(np.unwrap(np.angle(Ls)))
    pm = [(fs[i], 180 + ph[i]) for i in np.where(np.diff(np.sign(mag - 1)) != 0)[0]]
    gm = [(fs[i], -20 * np.log10(mag[i])) for i in np.where(np.diff(np.sign(ph + 180)) != 0)[0]]
    return pm, gm


def closed_loop_poles(gi, gv, R):
    """Poles of the full sampled loop (plant + both PIs + pipeline delays)."""
    Ad, Bd, Cav, Dav = plant(R)
    kpi, kii = gi[0], gi[1] * T
    kpv, kiv = gv[0], gv[1] * T
    # state: x(2), D[n], avg[n-1], integ_v, integ_i
    # cycle n: v = x2, iref = kpv e + Iv, Iv' = Iv + kiv e (e = -v),
    #          ei = iref - avg_prev, Ii' = Ii + kii ei, Dnext = kpi ei + Ii'
    #          avg_n = Cav x + Dav D, x' = Ad x + Bd D
    n = 6
    M = np.zeros((n, n))
    # x' = Ad x + Bd D
    M[0:2, 0:2] = Ad
    M[0:2, 2] = Bd[:, 0]
    # avg' (avg of this cycle) = Cav x + Dav D
    M[3, 0:2] = Cav[0]
    M[3, 2] = Dav[0, 0]
    # Iv' = Iv - kiv v
    M[4, 4] = 1
    M[4, 1] = -kiv
    # iref = kpv(-v) + Iv' = -(kpv + kiv) v + Iv
    iref = np.zeros(n)
    iref[1] = -(kpv + kiv)
    iref[4] = 1
    ei = iref.copy()
    ei[3] -= 1
    M[5, :] = np.eye(n)[5] + kii * ei
    M[2, :] = kpi * ei + M[5, :]
    return np.linalg.eigvals(M)


def main():
    f_ci = float(sys.argv[1]) if len(sys.argv) > 1 else 60e3
    f_cv = float(sys.argv[2]) if len(sys.argv) > 2 else 6e3
    z, gai, _ = resp(R_DESIGN, f_ci)
    gi = solve_pi(z ** -2 * gai, z)
    z = np.exp(1j * 2 * np.pi * f_cv * T)
    gv = solve_pi(lv(gi, (1.0, 0.0), R_DESIGN, f_cv) / pi_c(1.0, 0.0, z), z)
    print(f"current PI (f_ci = {f_ci/1e3:g} kHz): Kp = {gi[0]:.6g} 1/A, Ki = {gi[1]:.6g} 1/(A s)")
    print(f"voltage PI (f_cv = {f_cv/1e3:g} kHz): Kp = {gv[0]:.6g} A/V, Ki = {gv[1]:.6g} A/(V s)")
    for R in (R_DESIGN, 8.0, 0.8, 0.27):
        pmi, gmi = margins(lambda f: li(gi, R, f))
        pmv, gmv = margins(lambda f: lv(gi, gv, R, f))
        rho = max(abs(closed_loop_poles(gi, gv, R)))
        fmt = lambda l, u: ", ".join(f"{v:.1f}{u} @ {f/1e3:.1f} kHz" for f, v in l)
        print(f"R = {R:g} ohm: current PM {fmt(pmi, 'deg')}; GM {fmt(gmi, 'dB')}")
        print(f"            voltage PM {fmt(pmv, 'deg')}; GM {fmt(gmv, 'dB')}; max|pole| = {rho:.4f}")


if __name__ == "__main__":
    main()
