package frc.lib.catalyst.util;

import frc.lib.catalyst.hardware.CatalystMotor;

/**
 * Bundle of {@link TunableNumber}s for a mechanism's Slot-0 PID + feedforward
 * gains and, optionally, its Motion Magic profile constants. Hides the
 * boilerplate of registering every gain individually and re-applying them to
 * the underlying motor when any one changes.
 *
 * <p>Catalyst mechanisms (LinearMechanism, RotationalMechanism, FlywheelMechanism,
 * DifferentialWristMechanism) wire one of these in automatically — every PID/MM
 * gain is live-editable from the dashboard out of the box. Tunables are
 * published under {@code Catalyst/Tuning/<MechanismName>/...}.
 *
 * <p>For competition builds, call {@link TunableNumber#disableTuning()} once at
 * robot init. Every {@code hasChanged()} call will then return false, gains
 * stay at their initial values, and there is zero NT traffic.
 *
 * <p>Teams normally don't construct this directly — it's an internal helper
 * used by the built-in mechanisms.
 */
public final class TunableGains {

    private final TunableNumber kP, kI, kD;
    private final TunableNumber kS, kV, kA, kG;
    private final boolean hasMotionMagic;
    private final TunableNumber mmCruise, mmAccel, mmJerk;

    /** PID + feedforward only (no Motion Magic). */
    public TunableGains(String prefix,
                        double kP, double kI, double kD,
                        double kS, double kV, double kA, double kG) {
        this(prefix, kP, kI, kD, kS, kV, kA, kG, 0, 0, 0);
    }

    /** Full PID + feedforward + Motion Magic. */
    public TunableGains(String prefix,
                        double kP, double kI, double kD,
                        double kS, double kV, double kA, double kG,
                        double mmCruiseVelocity, double mmAcceleration, double mmJerk) {
        this.kP = new TunableNumber(prefix + "/kP", kP);
        this.kI = new TunableNumber(prefix + "/kI", kI);
        this.kD = new TunableNumber(prefix + "/kD", kD);
        this.kS = new TunableNumber(prefix + "/kS", kS);
        this.kV = new TunableNumber(prefix + "/kV", kV);
        this.kA = new TunableNumber(prefix + "/kA", kA);
        this.kG = new TunableNumber(prefix + "/kG", kG);

        this.hasMotionMagic = mmCruiseVelocity > 0 || mmAcceleration > 0 || mmJerk > 0;
        if (hasMotionMagic) {
            this.mmCruise = new TunableNumber(prefix + "/MM/CruiseVelocity", mmCruiseVelocity);
            this.mmAccel  = new TunableNumber(prefix + "/MM/Acceleration",  mmAcceleration);
            this.mmJerk   = new TunableNumber(prefix + "/MM/Jerk",          mmJerk);
        } else {
            this.mmCruise = null;
            this.mmAccel  = null;
            this.mmJerk   = null;
        }
    }

    /**
     * Re-apply Slot 0 and (if configured) Motion Magic constants to {@code motor}
     * whenever any tunable has changed since the last call. Safe and cheap to
     * call every loop — when nothing has changed this is just a few field
     * comparisons.
     */
    public void checkAndApply(CatalystMotor motor) {
        boolean slotChanged = kP.hasChanged() | kI.hasChanged() | kD.hasChanged()
                | kS.hasChanged() | kV.hasChanged() | kA.hasChanged() | kG.hasChanged();
        if (slotChanged) {
            motor.updateSlot0(kP.get(), kI.get(), kD.get(),
                              kS.get(), kV.get(), kA.get(), kG.get());
        }
        if (hasMotionMagic) {
            boolean mmChanged = mmCruise.hasChanged() | mmAccel.hasChanged() | mmJerk.hasChanged();
            if (mmChanged) {
                motor.updateMotionMagic(mmCruise.get(), mmAccel.get(), mmJerk.get());
            }
        }
    }

    /**
     * Re-apply gains to multiple motors that share the same Slot 0 / Motion
     * Magic config (e.g., the left + right motors of a differential wrist).
     */
    public void checkAndApply(CatalystMotor... motors) {
        for (CatalystMotor m : motors) {
            checkAndApply(m);
        }
    }
}
