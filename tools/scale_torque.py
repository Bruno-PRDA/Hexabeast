#!/usr/bin/env python3
"""How big a hexapod can a DT996-class servo carry?

The DT996 is an MG996R-form-factor digital metal-gear servo sold as 15 kg.cm
(180 deg, 55 g). Advertised: 13.5 kg.cm (1.32 N.m) at 4.8 V, 15.2 kg.cm
(1.49 N.m) at 6 V. Treat those as labels, not measurements - clones of this
class routinely test nearer a genuine MG996R's 11 kg.cm, which is exactly what
the 25 % design margin below is there to absorb.

This project's Phase-1 physics sim measured
the worst-case per-servo torque of the current design (foot span ~34 cm, all-up
1.32 kg): 0.39 N.m walking, 0.78 turning, 0.83 climbing a 5 deg ramp.

This sweeps the robot's linear scale k and asks where the DT996 runs out. The
key physics: the 18 leg servos are a FIXED 0.99 kg no matter how big the frame
gets, so mass grows slowly with size while the leg's lever arm grows linearly -
torque roughly tracks mass x reach. The torque constant is calibrated so k=1
reproduces the sim's measured 0.83 N.m, so this extrapolates a real datapoint
rather than a guess.

    python tools/scale_torque.py            # the table
    python tools/scale_torque.py --arm      # add the 6-servo arm's mass + payload
"""
import argparse

G = 9.81
SERVO_MASS = 0.055                 # one DT996/MG996R, kg
N_LEG_SERVOS = 18
N_ARM_SERVOS = 6
BATT_ELEC = 0.30                   # 2S LiPo + ESP32 + camera + 2x PCA9685 + wiring, fixed
STRUCT_1 = 0.25                    # frame + brackets + links at k=1, scales as k^3
REACH_1 = 0.095                    # horizontal coxa-pivot -> foot at k=1, m
HIP_OFFSET_1 = 0.075               # body centre -> coxa pivot at k=1, m
SIM_CLIMB_TORQUE = 0.83            # per-servo worst case measured at k=1, N.m

# DT996 advertised stall torque by supply voltage, N.m.
RATING = {"4.8 V": 1.32, "6 V": 1.49}
# Fraction worth designing to. Covers stall spikes, gear life, and the gap
# between a clone's advertised torque and its real one.
USABLE = 0.75


def mass(k, arm=False):
    servos = (N_LEG_SERVOS + (N_ARM_SERVOS if arm else 0)) * SERVO_MASS
    return servos + BATT_ELEC + STRUCT_1 * k ** 3


# Calibrate the lumped constant C (weight fraction on the worst leg x lever
# fraction) so the model reproduces the sim's measured climbing torque at k=1.
C = SIM_CLIMB_TORQUE / (mass(1.0) * G * REACH_1)


def climb_torque(k, arm=False):
    return C * mass(k, arm) * G * (REACH_1 * k)


def foot_span(k):
    return 2.0 * (HIP_OFFSET_1 + REACH_1) * k     # foot-to-foot diameter, m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", action="store_true", help="include the front arm's 6 servos in the mass")
    args = ap.parse_args()

    print(f"DT996-class servo, calibrated to the sim's {SIM_CLIMB_TORQUE} N.m climb peak at k=1.")
    print(f"Usable torque = {USABLE:.0%} of stall: {RATING['4.8 V']*USABLE:.2f} N.m at 4.8 V, "
          f"{RATING['6 V']*USABLE:.2f} N.m at 6 V.")
    print(f"Arm mass {'INCLUDED' if args.arm else 'excluded'}.\n")
    print(f"{'scale':>5} {'foot span':>10} {'body len':>9} {'mass':>7} {'climb tau':>10}   verdict")
    print("-" * 62)
    for k in (0.7, 0.85, 1.0, 1.15, 1.3, 1.5, 1.75, 2.0):
        tau = climb_torque(k, args.arm)
        if tau <= RATING["4.8 V"] * USABLE:
            verdict = "comfortable (even at 4.8 V)"
        elif tau <= RATING["6 V"] * USABLE:
            verdict = "OK at 6 V, tight on obstacles"
        elif tau <= RATING["6 V"]:
            verdict = "walks flat only, stalls climbing"
        else:
            verdict = "TOO BIG for DT996"
        print(f"{k:>5.2f} {foot_span(k)*100:>8.0f} cm {0.18*k*100:>6.0f} cm "
              f"{mass(k, args.arm):>6.2f} kg {tau:>8.2f} N.m   {verdict}")

    # Largest comfortable and largest workable spans, by bisection.
    def max_k(limit):
        lo, hi = 0.3, 4.0
        for _ in range(60):
            mid = (lo + hi) / 2
            if climb_torque(mid, args.arm) <= limit:
                lo = mid
            else:
                hi = mid
        return lo

    print(f"\nLargest COMFORTABLE size on DT996: foot span {foot_span(max_k(RATING['6 V']*USABLE))*100:.0f} cm "
          f"(6 V, 25 % margin).")
    print(f"Absolute ceiling before stall while climbing: {foot_span(max_k(RATING['6 V']))*100:.0f} cm.")

    if args.arm:
        # The arm's own shoulder servo is the other DT996 limit: it holds the
        # whole arm + payload out at full reach.
        arm_reach = 0.24
        arm_link_mass = 0.20
        payload = RATING["6 V"] * USABLE / (G * arm_reach) - arm_link_mass
        print(f"\nArm shoulder (DT996 at 6 V): max payload at {arm_reach*100:.0f} cm full reach "
              f"~ {payload*1000:.0f} g. Fold the arm to lift more.")


if __name__ == "__main__":
    main()
