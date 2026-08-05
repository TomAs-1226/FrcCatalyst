package frc.lib.catalyst.physics.sim;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.catalyst.physics.PhysicsSample;

/**
 * Corrupts simulated samples on purpose, so the detectors can be tested against faults that are
 * otherwise hard to produce.
 *
 * <p>Validating a slip detector on a real robot means finding a slick patch of carpet and spinning
 * the wheels at exactly the right moment, repeatedly, and hoping the log captured it. Validating a
 * collision detector means asking somebody to hit the robot. Neither is a test you can run a hundred
 * times, and neither is a test you can run before the robot exists.
 *
 * <p>This injects the fault instead. Wrap a clean sample stream, turn on the fault you want, and check
 * that the detector notices — deterministically, in a unit test, with no field and no robot.
 *
 * <pre>{@code
 * DisturbanceInjector injector = DisturbanceInjector.builder()
 *     .slipModule(2, 2.5)                 // module 2 reads 2.5 m/s fast
 *     .impact(new Translation2d(0, -14))  // 14 m/s^2 sideways the wheels never commanded
 *     .build();
 *
 * PhysicsSample corrupted = injector.apply(cleanSample);
 * physics.update(corrupted);
 * assertTrue(physics.analyze().isSlipping());
 * }</pre>
 *
 * <p>Faults can be switched on and off between samples, which is how you test that a detector both
 * fires and <em>clears</em> — the second half being the one that usually gets skipped.
 *
 * <p>Sim and test only. Nothing here belongs in competition code, and nothing in Physics Core reaches
 * for it.
 *
 * @since 1.6.0
 */
public final class DisturbanceInjector {

    private final List<ModuleSlip> moduleSlips;
    private Translation2d impactAcceleration;
    private double wheelSpeedBias;
    private boolean enabled = true;

    private DisturbanceInjector(Builder builder) {
        this.moduleSlips = new ArrayList<>(builder.moduleSlips);
        this.impactAcceleration = builder.impactAcceleration;
        this.wheelSpeedBias = builder.wheelSpeedBias;
    }

    /**
     * Apply every enabled fault to a sample, returning a corrupted copy. The input is not modified,
     * so the clean stream stays available for comparison.
     */
    public PhysicsSample apply(PhysicsSample sample) {
        if (!enabled) return sample;

        SwerveModuleState[] states = sample.moduleStates();
        if (states != null && (!moduleSlips.isEmpty() || wheelSpeedBias != 0.0)) {
            states = states.clone();
            for (int i = 0; i < states.length; i++) {
                double extra = wheelSpeedBias;
                for (ModuleSlip slip : moduleSlips) {
                    if (slip.index == i) extra += slip.extraSpeedMps;
                }
                if (extra != 0.0) {
                    states[i] = new SwerveModuleState(
                            states[i].speedMetersPerSecond + extra, states[i].angle);
                }
            }
        }

        // A real slip also inflates the forward-kinematic chassis speed, because that is derived from
        // the same wheels. Injecting only the module states would produce a fault no real robot has.
        ChassisSpeeds speeds = sample.robotRelativeSpeeds();
        if (wheelSpeedBias != 0.0) {
            speeds = new ChassisSpeeds(speeds.vxMetersPerSecond + wheelSpeedBias,
                    speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        }

        Translation2d acceleration = sample.robotRelativeAcceleration();
        if (impactAcceleration != null) {
            acceleration = acceleration == null ? impactAcceleration : acceleration.plus(impactAcceleration);
        }

        return new PhysicsSample(sample.timestampSeconds(), sample.pose(), speeds, states,
                acceleration, sample.yawRateRadPerSec());
    }

    /**
     * Start or stop an impact between samples. Passing {@code null} clears it, which is how you test
     * that a collision detector goes quiet again afterwards.
     *
     * @param robotRelativeAcceleration acceleration the wheels never commanded, m/s^2
     */
    public void setImpact(Translation2d robotRelativeAcceleration) {
        this.impactAcceleration = robotRelativeAcceleration;
    }

    /** Add speed to every wheel, simulating a whole drivetrain breaking traction at once. */
    public void setWheelSpeedBias(double wheelSpeedBias) {
        this.wheelSpeedBias = wheelSpeedBias;
    }

    /** Turn every fault on or off at once, leaving the configuration in place. */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /** Whether any fault is currently being applied. */
    public boolean isEnabled() {
        return enabled;
    }

    /** Remove every configured fault. */
    public void clear() {
        moduleSlips.clear();
        impactAcceleration = null;
        wheelSpeedBias = 0.0;
    }

    /** One module reporting more speed than it is delivering. */
    private record ModuleSlip(int index, double extraSpeedMps) {}

    /** Start building an injector. */
    public static Builder builder() {
        return new Builder();
    }

    /** Builder for {@link DisturbanceInjector}. */
    public static final class Builder {
        private final List<ModuleSlip> moduleSlips = new ArrayList<>();
        private Translation2d impactAcceleration;
        private double wheelSpeedBias = 0.0;

        /**
         * Make one module report faster than it is really moving — a single wheel breaking traction,
         * or one in the air.
         *
         * @param moduleIndex   which module, in kinematics order
         * @param extraSpeedMps how much faster it reads, in m/s
         */
        public Builder slipModule(int moduleIndex, double extraSpeedMps) {
            if (moduleIndex < 0) {
                throw new IllegalArgumentException("moduleIndex must be >= 0 (got " + moduleIndex + ")");
            }
            moduleSlips.add(new ModuleSlip(moduleIndex, extraSpeedMps));
            return this;
        }

        /** Make every wheel read fast — the whole drivetrain losing grip together. */
        public Builder slipAllWheels(double extraSpeedMps) {
            this.wheelSpeedBias = extraSpeedMps;
            return this;
        }

        /**
         * Add acceleration the wheels never commanded — being hit, or hitting something.
         *
         * @param robotRelativeAcceleration the impulse, in m/s^2, robot-relative
         */
        public Builder impact(Translation2d robotRelativeAcceleration) {
            this.impactAcceleration = robotRelativeAcceleration;
            return this;
        }

        /** Build. Every fault is optional; an injector with none configured is a no-op. */
        public DisturbanceInjector build() {
            return new DisturbanceInjector(this);
        }
    }
}
