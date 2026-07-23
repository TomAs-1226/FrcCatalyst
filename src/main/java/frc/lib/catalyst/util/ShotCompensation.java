package frc.lib.catalyst.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Live, operator-adjustable aim bias for a turreted shooter — the knobs
 * you reach for when the static tables are slightly off, the field
 * element has drifted, or a defender is bumping you and you want to dial
 * the shoot-on-the-fly aggressiveness down.
 *
 * <p>An {@link AimingSolverTOF} optionally consumes one of these and applies
 * it on top of its computed solution:
 * <ul>
 *   <li><b>turret bias</b> — degrees added to the final aim bearing</li>
 *   <li><b>distance bias</b> — metres added to the lookup distance (shoots
 *       longer/shorter without moving the turret)</li>
 *   <li><b>RPM / hood bias</b> — added to the looked-up shooter values</li>
 *   <li><b>velocity scale</b> — multiplies the SOTF correction. 1.0 = full
 *       motion compensation, 0.0 = aim as if stationary. Dial down when
 *       the velocity estimate is noisy (e.g. while being shoved).</li>
 * </ul>
 *
 * <p>Two robustness limits guard against a defender's hit corrupting the
 * solve:
 * <ul>
 *   <li><b>velocity deadband</b> — chassis speeds below this are treated as
 *       zero, so sensor noise doesn't wiggle the aim while you're parked.</li>
 *   <li><b>max compensated speed</b> — the speed used for motion comp is
 *       clamped to this, so a collision spike in the velocity estimate
 *       can't fling the turret to a wild angle.</li>
 * </ul>
 *
 * <p>Code-authoritative: bind the {@code nudge*} methods to operator
 * buttons (a D-pad is the classic choice). Current values publish to
 * {@code /Catalyst/Aiming/<name>/...} for the dashboard.
 *
 * <pre>{@code
 * ShotCompensation comp = ShotCompensation.builder()
 *     .name("Turret")
 *     .velocityDeadband(0.08)
 *     .maxCompensatedSpeed(4.0)
 *     .build();
 *
 * AimingSolver solver = AimingSolver.builder()
 *     .target(GOAL).shotTime(shotTimeTable)
 *     .compensation(comp)
 *     .build();
 *
 * operator.povUp()  .onTrue(Commands.runOnce(() -> comp.nudgeHoodBias(+1)));
 * operator.povDown().onTrue(Commands.runOnce(() -> comp.nudgeHoodBias(-1)));
 * operator.povRight().onTrue(Commands.runOnce(() -> comp.nudgeTurretBias(+0.5)));
 * operator.povLeft() .onTrue(Commands.runOnce(() -> comp.nudgeTurretBias(-0.5)));
 * operator.start()  .onTrue(Commands.runOnce(comp::reset));
 *
 * // Being defended? halve the SOTF correction:
 * operator.leftBumper().onTrue (Commands.runOnce(() -> comp.setVelocityScale(0.5)));
 * operator.leftBumper().onFalse(Commands.runOnce(() -> comp.setVelocityScale(1.0)));
 * }</pre>
 */
public final class ShotCompensation {

    private double turretBiasDeg;
    private double distanceBiasMeters;
    private double rpmBias;
    private double hoodBiasDeg;
    private double velocityScale;
    private final double velocityDeadband;
    private final double maxCompensatedSpeed;

    private final NetworkTable nt;

    private ShotCompensation(Builder b) {
        this.turretBiasDeg = b.turretBiasDeg;
        this.distanceBiasMeters = b.distanceBiasMeters;
        this.rpmBias = b.rpmBias;
        this.hoodBiasDeg = b.hoodBiasDeg;
        this.velocityScale = b.velocityScale;
        this.velocityDeadband = b.velocityDeadband;
        this.maxCompensatedSpeed = b.maxCompensatedSpeed;
        this.nt = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Aiming").getSubTable(b.name);
        publish();
    }

    // ----- live setters -----

    public void setTurretBias(double degrees)      { this.turretBiasDeg = degrees; publish(); }
    public void setDistanceBias(double meters)     { this.distanceBiasMeters = meters; publish(); }
    public void setRpmBias(double rpm)             { this.rpmBias = rpm; publish(); }
    public void setHoodBias(double degrees)        { this.hoodBiasDeg = degrees; publish(); }

    /** SOTF aggressiveness. Clamped to [0, 2]. 1.0 = full motion comp. */
    public void setVelocityScale(double scale)     { this.velocityScale = clamp(scale, 0, 2); publish(); }

    // ----- nudges (bind to buttons) -----

    public void nudgeTurretBias(double deltaDeg)   { setTurretBias(turretBiasDeg + deltaDeg); }
    public void nudgeDistanceBias(double deltaM)   { setDistanceBias(distanceBiasMeters + deltaM); }
    public void nudgeRpmBias(double deltaRpm)      { setRpmBias(rpmBias + deltaRpm); }
    public void nudgeHoodBias(double deltaDeg)     { setHoodBias(hoodBiasDeg + deltaDeg); }

    /** Zero every bias and restore full motion compensation. */
    public void reset() {
        turretBiasDeg = 0;
        distanceBiasMeters = 0;
        rpmBias = 0;
        hoodBiasDeg = 0;
        velocityScale = 1.0;
        publish();
    }

    // ----- getters used by AimingSolver -----

    public double turretBiasDeg()      { return turretBiasDeg; }
    public double distanceBiasMeters() { return distanceBiasMeters; }
    public double rpmBias()            { return rpmBias; }
    public double hoodBiasDeg()        { return hoodBiasDeg; }
    public double velocityScale()      { return velocityScale; }
    public double velocityDeadband()   { return velocityDeadband; }
    public double maxCompensatedSpeed(){ return maxCompensatedSpeed; }

    /**
     * Condition a single velocity axis for motion compensation: zero it
     * inside the deadband, clamp its magnitude to the max, then scale.
     * The solver calls this on vx and vy.
     */
    public double conditionVelocity(double v) {
        double out = Math.abs(v) < velocityDeadband ? 0.0 : v;
        out = clamp(out, -maxCompensatedSpeed, maxCompensatedSpeed);
        return out * velocityScale;
    }

    private void publish() {
        nt.getEntry("TurretBiasDeg").setDouble(turretBiasDeg);
        nt.getEntry("DistanceBiasMeters").setDouble(distanceBiasMeters);
        nt.getEntry("RpmBias").setDouble(rpmBias);
        nt.getEntry("HoodBiasDeg").setDouble(hoodBiasDeg);
        nt.getEntry("VelocityScale").setDouble(velocityScale);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ============================================================
    //                        BUILDER
    // ============================================================

    public static Builder builder() { return new Builder(); }

    public static class Builder {
        private String name = "Turret";
        private double turretBiasDeg = 0;
        private double distanceBiasMeters = 0;
        private double rpmBias = 0;
        private double hoodBiasDeg = 0;
        private double velocityScale = 1.0;
        private double velocityDeadband = 0.05;   // m/s
        private double maxCompensatedSpeed = 4.5; // m/s

        /** NT subtable name under /Catalyst/Aiming/. Default "Turret". */
        public Builder name(String name) { this.name = name; return this; }

        public Builder turretBias(double degrees) { this.turretBiasDeg = degrees; return this; }
        public Builder distanceBias(double meters) { this.distanceBiasMeters = meters; return this; }
        public Builder rpmBias(double rpm) { this.rpmBias = rpm; return this; }
        public Builder hoodBias(double degrees) { this.hoodBiasDeg = degrees; return this; }

        /** Initial SOTF scale, [0, 2]. Default 1.0. */
        public Builder velocityScale(double scale) { this.velocityScale = clamp(scale, 0, 2); return this; }

        /** Chassis speed below this (m/s) is ignored for motion comp. Default 0.05. */
        public Builder velocityDeadband(double mps) { this.velocityDeadband = Math.max(0, mps); return this; }

        /** Motion-comp speed is clamped to this magnitude (m/s). Default 4.5. */
        public Builder maxCompensatedSpeed(double mps) { this.maxCompensatedSpeed = Math.max(0.1, mps); return this; }

        public ShotCompensation build() { return new ShotCompensation(this); }
    }
}
