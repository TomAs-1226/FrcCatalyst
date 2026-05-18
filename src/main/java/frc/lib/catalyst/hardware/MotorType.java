package frc.lib.catalyst.hardware;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Specification for an FRC motor — torque, free speed, current draw.
 *
 * <p>Used for simulation models, motion-profile calculation
 * ({@link frc.lib.catalyst.util.MotionConstraintCalculator}), gravity
 * feedforward estimation, and any helper that needs to reason about
 * what the motor can physically deliver.
 *
 * <p>This was an {@code enum} prior to v0.3.3-beta but is now a regular
 * {@code final class}. The existing static constants below
 * ({@code MotorType.KRAKEN_X60} etc.) still work exactly as before, but
 * teams can also declare their own:
 *
 * <pre>{@code
 * // For NEO 550, Minion, or any motor Catalyst doesn't ship a preset for:
 * MotorType neo550 = new MotorType(
 *     "NEO 550",
 *     0.97,    // stall torque (Nm)
 *     11000,   // free speed (RPM)
 *     100,     // stall current (A)
 *     1.4);    // free current (A)
 * }</pre>
 *
 * <p><b>FOC variants:</b> Phoenix-6 FOC mode delivers ~30% more stall
 * torque than non-FOC at the cost of a small free-speed reduction.
 * Always pick the variant that matches what you actually configure on
 * the TalonFX — using a non-FOC preset on an FOC-driven motor will
 * under-state torque in sim and over-state required holding voltage.
 *
 * <p>Specs are sourced from CTRE's published motor pages and match the
 * values in {@code DCMotor.getKrakenX60()} / {@code .getKrakenX60Foc()} /
 * {@code .getFalcon500()} / {@code .getFalcon500Foc()} in WPILib 2026.
 */
public final class MotorType {

    /** Kraken X60, no FOC (Phoenix 6 default duty-cycle / voltage control). */
    public static final MotorType KRAKEN_X60     = new MotorType("Kraken X60",     7.09, 6000, 366, 2.0);
    /** Kraken X60 with FOC enabled — ~30% more stall torque, slightly less free speed. */
    public static final MotorType KRAKEN_X60_FOC = new MotorType("Kraken X60 FOC", 9.37, 5800, 483, 2.0);
    /** Kraken X44 (NEMA-style chassis motor), no FOC. */
    public static final MotorType KRAKEN_X44     = new MotorType("Kraken X44",     4.05, 7530, 275, 1.4);
    /** Kraken X44 with FOC enabled. */
    public static final MotorType KRAKEN_X44_FOC = new MotorType("Kraken X44 FOC", 5.45, 7200, 366, 1.4);
    /** Falcon 500 (legacy, pre-2024), no FOC. */
    public static final MotorType FALCON_500     = new MotorType("Falcon 500",     4.69, 6380, 257, 1.5);
    /** Falcon 500 with FOC enabled (Phoenix Pro / Phoenix 6). */
    public static final MotorType FALCON_500_FOC = new MotorType("Falcon 500 FOC", 5.84, 6080, 304, 1.5);

    // ---- Non-CTRE motors ----------------------------------------------------
    // FrcCatalyst is CTRE-first, but a few presets are shipped for teams who
    // mix in REV or use the WCP Minion on non-drive mechanisms. They do not
    // gain Phoenix-6 features (FOC, fused CANcoder, etc.) — they're here so
    // sim, gravity feedforward, and MotionConstraintCalculator work for teams
    // running a mixed motor stack.

    /** REV NEO Brushless v1 (legacy NEO). */
    public static final MotorType NEO         = new MotorType("NEO",         2.6,  5676, 105, 1.8);
    /** REV NEO Vortex (brushless successor with stiffer torque curve). */
    public static final MotorType NEO_VORTEX  = new MotorType("NEO Vortex",  3.6,  6784, 211, 3.6);
    /** REV NEO 550 (compact gripper / intake motor). */
    public static final MotorType NEO_550     = new MotorType("NEO 550",     0.97, 11000, 100, 1.4);
    /** WCP Minion (brushless, CTRE-controllable via SparkMax — drive-class torque in a smaller package). */
    public static final MotorType MINION      = new MotorType("Minion",      3.1,  7200, 200, 1.5);

    /** Human-readable name, used in logs and the Catalyst Tuner UI. */
    public final String displayName;
    /** Stall torque per motor (Nm) at the spec'd nominal voltage. */
    public final double stallTorqueNm;
    /** Free (no-load) speed (RPM) at the spec'd nominal voltage. */
    public final double freeSpeedRPM;
    /** Stall current per motor (A). */
    public final double stallCurrentAmps;
    /** Free (no-load) current per motor (A). */
    public final double freeCurrentAmps;
    /** Nominal supply voltage the above figures are referenced to (default 12 V). */
    public final double nominalVoltage;

    /** Construct a 12 V motor spec. Most FRC motors use 12 V — call this. */
    public MotorType(String displayName, double stallTorqueNm, double freeSpeedRPM,
                     double stallCurrentAmps, double freeCurrentAmps) {
        this(displayName, stallTorqueNm, freeSpeedRPM, stallCurrentAmps, freeCurrentAmps, 12.0);
    }

    /** Full constructor — only needed for non-12V motor specs. */
    public MotorType(String displayName, double stallTorqueNm, double freeSpeedRPM,
                     double stallCurrentAmps, double freeCurrentAmps, double nominalVoltage) {
        this.displayName = displayName;
        this.stallTorqueNm = stallTorqueNm;
        this.freeSpeedRPM = freeSpeedRPM;
        this.stallCurrentAmps = stallCurrentAmps;
        this.freeCurrentAmps = freeCurrentAmps;
        this.nominalVoltage = nominalVoltage;
    }

    /** Free speed in rotations per second. */
    public double freeSpeedRPS() {
        return freeSpeedRPM / 60.0;
    }

    /** Free speed in radians per second. */
    public double freeSpeedRadPerSec() {
        return freeSpeedRPM * 2.0 * Math.PI / 60.0;
    }

    /**
     * Build the WPILib {@link DCMotor} model for {@code numMotors} of this
     * motor ganged on one shaft. Uses the motor's own spec values rather
     * than relying on a switch over {@code DCMotor.getKrakenX60()} et al.,
     * so user-declared motors work the same as the built-in presets.
     */
    public DCMotor getDCMotor(int numMotors) {
        return new DCMotor(
                nominalVoltage,
                stallTorqueNm,
                stallCurrentAmps,
                freeCurrentAmps,
                freeSpeedRadPerSec(),
                numMotors);
    }

    /** Max mechanism RPM given a motor-to-mechanism gear reduction. */
    public double maxMechanismRPM(double gearRatio) {
        return freeSpeedRPM / gearRatio;
    }

    /** Peak stall torque at the output shaft (Nm) for {@code motorCount} motors through a reduction. */
    public double maxMechanismTorque(double gearRatio, int motorCount) {
        return stallTorqueNm * gearRatio * motorCount;
    }

    /**
     * Linear estimate of the supply voltage required to hold a load that
     * applies {@code loadTorqueNm} at the mechanism through a {@code gearRatio}
     * reduction. Useful for gravity feedforward on arms and elevators.
     *
     * <p>The estimate is a torque-fraction model and ignores back-EMF / friction;
     * it's a starting point, not a final tune.
     */
    public double holdingVoltage(double loadTorqueNm, double gearRatio) {
        double motorTorque = loadTorqueNm / gearRatio;
        return (motorTorque / stallTorqueNm) * nominalVoltage;
    }

    @Override
    public String toString() {
        return displayName;
    }
}
