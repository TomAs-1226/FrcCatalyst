package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

/**
 * Hardware-independent aiming math for a turreted shooter, including Shoot-On-The-Fly (SOTF) motion
 * compensation, solved by 3D vector addition.
 *
 * <p>This is an alternative to {@link AimingSolver}, and the two answer different questions.
 * {@code AimingSolver} is the time-of-flight solver: it leads a moving virtual goal and hands back a
 * field-relative turret bearing and bearing rate. {@code AimingSolverVector} instead treats the shot
 * as a 3D velocity vector, subtracts the robot's field-relative velocity from it, and returns the
 * hood pitch, yaw and flywheel RPS that put the resulting vector back on the target — so it also
 * solves the hood angle and wheel speed, not just the bearing. Pick whichever matches how your
 * shooter is characterised. Like {@code AimingSolver}, it touches no motors and no NetworkTables; it
 * is pure math and is unit-testable with no hardware.
 *
 * <p>The vector formulation follows the Huskie Robotics write-up on
 * <a href="https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805">
 * shoot-on-the-move with equations</a>.
 *
 * <p>Contributed by <a href="https://github.com/avrahamavraham">avrahamavraham</a> (PR #27).
 *
 * @since 1.2.1
 */
public class AimingSolverVector {

    /**
     * The outputs of a solve: where to point the hood and turret, and how fast to spin the wheel.
     *
     * @param hoodPitch required hood elevation
     * @param yaw       required field-relative turret bearing
     * @param rps       required flywheel speed in rotations per second
     */
    public static record TargetState(
            Rotation2d hoodPitch,
            Rotation2d yaw,
            double rps) {}

    private final InterpolatingDoubleTreeMap rpsMap;
    private final InterpolatingDoubleTreeMap hoodMap;
    private final Translation2d targetPosition;
    private final Rotation2d minHoodAngle;
    private final Rotation2d maxHoodAngle;
    private final double wheelDiameterMeters;
    private final double efficiency;

    private AimingSolverVector(Builder builder) {
        this.rpsMap = builder.rpsMap;
        this.hoodMap = builder.hoodMap;
        this.targetPosition = builder.targetPosition;
        this.minHoodAngle = builder.minHoodAngle;
        this.maxHoodAngle = builder.maxHoodAngle;
        this.wheelDiameterMeters = builder.wheelDiameterMeters;
        this.efficiency = builder.efficiency;
    }

    /**
     * Calculate the required hood pitch, yaw, and flywheel RPS to hit the target while driving.
     *
     * @param robotPose           current field-relative pose of the robot
     * @param fieldRelativeSpeeds current field-relative velocity of the robot chassis
     * @return the required hood pitch, yaw (field-relative bearing), and flywheel RPS
     */
    public TargetState calculate(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
        Translation2d robotTranslation = robotPose.getTranslation();
        double distanceMeters = robotTranslation.getDistance(targetPosition);

        double staticRps = rpsMap.get(distanceMeters);
        Rotation2d staticHoodPitch = Rotation2d.fromDegrees(hoodMap.get(distanceMeters));

        double wheelCircumference = Math.PI * wheelDiameterMeters;
        double staticSpeedMps = staticRps * wheelCircumference * efficiency;

        double theta = (Math.PI / 2.0) - staticHoodPitch.getRadians();
        double phi = targetPosition.minus(robotTranslation).getAngle().getRadians();

        double vx = staticSpeedMps * Math.sin(theta) * Math.cos(phi);
        double vy = staticSpeedMps * Math.sin(theta) * Math.sin(phi);
        double vz = staticSpeedMps * Math.cos(theta);

        double reqVx = vx - fieldRelativeSpeeds.vxMetersPerSecond;
        double reqVy = vy - fieldRelativeSpeeds.vyMetersPerSecond;
        double reqVz = vz;

        double reqPhi = Math.atan2(reqVy, reqVx);
        double reqTheta = Math.atan2(Math.hypot(reqVx, reqVy), reqVz);
        double reqHoodElevationRad = (Math.PI / 2.0) - reqTheta;

        Rotation2d calculatedHoodPitch = new Rotation2d(reqHoodElevationRad);
        Rotation2d finalHoodPitch;
        double finalSpeedMps;

        // Clamp against the configured hood limits. When the pitch is clamped, the wheel speed is
        // re-derived so the horizontal component of the shot is preserved at the limit angle.
        if (calculatedHoodPitch.getDegrees() < minHoodAngle.getDegrees()) {
            finalHoodPitch = minHoodAngle;
            double clampedTheta = (Math.PI / 2.0) - finalHoodPitch.getRadians();
            double reqVhoriz = Math.hypot(reqVx, reqVy);
            finalSpeedMps = reqVhoriz / Math.sin(clampedTheta);
        } else if (calculatedHoodPitch.getDegrees() > maxHoodAngle.getDegrees()) {
            finalHoodPitch = maxHoodAngle;
            double clampedTheta = (Math.PI / 2.0) - finalHoodPitch.getRadians();
            double reqVhoriz = Math.hypot(reqVx, reqVy);
            finalSpeedMps = reqVhoriz / Math.sin(clampedTheta);
        } else {
            finalHoodPitch = calculatedHoodPitch;
            finalSpeedMps = Math.sqrt((reqVx * reqVx) + (reqVy * reqVy) + (reqVz * reqVz));
        }

        double finalRps = finalSpeedMps / (wheelCircumference * efficiency);
        Rotation2d finalYaw = new Rotation2d(reqPhi);

        return new TargetState(finalHoodPitch, finalYaw, finalRps);
    }

    /** Builder for {@link AimingSolverVector}. */
    public static class Builder {
        private InterpolatingDoubleTreeMap rpsMap;
        private InterpolatingDoubleTreeMap hoodMap;
        private Translation2d targetPosition;
        private Rotation2d minHoodAngle = Rotation2d.fromDegrees(0.0);
        private Rotation2d maxHoodAngle = Rotation2d.fromDegrees(90.0);
        private double wheelDiameterMeters = Units.inchesToMeters(3.0);
        private double efficiency = 1.0;

        /** Distance (m) to flywheel RPS lookup, from a shooter characterization sweep. */
        public Builder rpsMap(InterpolatingDoubleTreeMap rpsMap) {
            this.rpsMap = rpsMap;
            return this;
        }

        /** Distance (m) to hood pitch (degrees) lookup, from the same sweep. */
        public Builder hoodMap(InterpolatingDoubleTreeMap hoodMap) {
            this.hoodMap = hoodMap;
            return this;
        }

        /** Field position of the target (goal) to aim at. */
        public Builder targetPosition(Translation2d targetPosition) {
            this.targetPosition = targetPosition;
            return this;
        }

        /** Mechanical hood travel limits; a clamped solve re-derives wheel speed at the limit. */
        public Builder hoodLimits(Rotation2d minAngle, Rotation2d maxAngle) {
            this.minHoodAngle = minAngle;
            this.maxHoodAngle = maxAngle;
            return this;
        }

        /**
         * Convenience for a fixed-hood shooter: pins the hood map to one angle at every distance and
         * sets the travel limits to that same angle, so the solve only moves the turret and wheel.
         */
        public Builder staticHood(Rotation2d staticHoodPitch) {
            this.hoodMap = new InterpolatingDoubleTreeMap();
            this.hoodMap.put(0.0, staticHoodPitch.getDegrees());
            this.hoodMap.put(100.0, staticHoodPitch.getDegrees());
            return hoodLimits(staticHoodPitch, staticHoodPitch);
        }

        /** Flywheel wheel diameter in inches. */
        public Builder wheelDiameterInches(double diameterInches) {
            this.wheelDiameterMeters = Units.inchesToMeters(diameterInches);
            return this;
        }

        /** Flywheel wheel diameter in metres. */
        public Builder wheelDiameterMeters(double diameterMeters) {
            this.wheelDiameterMeters = diameterMeters;
            return this;
        }

        /** Exit-velocity efficiency: fraction of the surface speed the game piece actually leaves at. */
        public Builder efficiency(double efficiency) {
            this.efficiency = efficiency;
            return this;
        }

        /** Validate and build. Throws if a required map or the target position is missing. */
        public AimingSolverVector build() {
            if (rpsMap == null || hoodMap == null) {
                throw new IllegalStateException("Both RPS map and Hood map must be provided!");
            }
            if (targetPosition == null) {
                throw new IllegalStateException("A target position must be provided!");
            }
            return new AimingSolverVector(this);
        }
    }
}
