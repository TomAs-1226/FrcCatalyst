package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Threads;

/**
 *  * Hardware-independent aiming math for a turreted shooter, including
 * Shoot-On-The-Fly (SOTF) motion compensation.
 * 
 * this class dose not interact with motors or networks tables so it is only pure math and code
 * the class was created after this <a href="https://www.chiefdelphi.com/t/huskie-physics-shoot-on-the-move-with-equations/522805">post</a>.
 * 
 * 
 * 
    */
    public class AimingSolverVector {

        /**
         * Container for the outputs calculated for the shooter and drivetrain.
         */
        public static record TargetState(
            Rotation2d hoodPitch,
            Rotation2d yaw,
            double rps){}

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
         * Calculates the required hood pitch, yaw, and flywheel RPS to hit the target while driving.
         *
         * @param robotPose           Current field-relative pose of the robot.
         * @param fieldRelativeSpeeds Current field-relative velocity of the robot chassis.
         * @return TargetState containing required Hood Pitch, Yaw (Heading), and Flywheel RPS.
         */
        public TargetState calculate(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
            Translation2d robotTranslation = robotPose.getTranslation();
            double distanceMeters = robotTranslation.getDistance(targetPosition);
            double staticRps = rpsMap.get(distanceMeters);
            Rotation2d staticHoodPitch = Rotation2d.fromRotations(hoodMap.get(distanceMeters));
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

        /**
         * Builder class for constructing {@link AimingSolverVector} instances.
         */
        public static class Builder {
            private InterpolatingDoubleTreeMap rpsMap;
            private InterpolatingDoubleTreeMap hoodMap;
            private Translation2d targetPosition;
            private Rotation2d minHoodAngle = Rotation2d.fromDegrees(0.0);
            private Rotation2d maxHoodAngle = Rotation2d.fromDegrees(90.0);
            private double wheelDiameterMeters = Units.inchesToMeters(3.0); // Default 3 inches
            private double efficiency = 1.0; // Default 100% efficiency

            public Builder rpsMap(InterpolatingDoubleTreeMap rpsMap) {
                this.rpsMap = rpsMap;
                return this;
            }

            public Builder hoodMap(InterpolatingDoubleTreeMap hoodMap) {
                this.hoodMap = hoodMap;
                return this;
            }

            public Builder targetPosition(Translation2d targetPosition) {
                this.targetPosition = targetPosition;
                return this;
            }

            public Builder hoodLimits(Rotation2d minAngle, Rotation2d maxAngle) {
                this.minHoodAngle = minAngle;
                this.maxHoodAngle = maxAngle;
                return this;
            }

            public Builder StaticHood(Rotation2d StaticHoodPitch){
                this.hoodMap = new InterpolatingDoubleTreeMap();
                hoodMap.put(0.0, 0.0);
                return hoodLimits(StaticHoodPitch, StaticHoodPitch);
            }

            public Builder wheelDiameterInches(double diameterInches) {
                this.wheelDiameterMeters = Units.inchesToMeters(diameterInches);
                return this;
            }

            public Builder wheelDiameterMeters(double diameterMeters) {
                this.wheelDiameterMeters = diameterMeters;
                return this;
            }

            public Builder efficiency(double efficiency) {
                this.efficiency = efficiency;
                return this;
            }

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