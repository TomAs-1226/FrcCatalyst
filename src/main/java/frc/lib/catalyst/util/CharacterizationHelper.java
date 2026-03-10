package frc.lib.catalyst.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.catalyst.hardware.CatalystMotor;

import static edu.wpi.first.units.Units.*;

/**
 * Helper for creating WPILib SysId characterization routines for mechanisms.
 * SysId measures system dynamics (kS, kV, kA) which are needed for accurate
 * feedforward control.
 *
 * <p>Example usage:
 * <pre>{@code
 * CharacterizationHelper charHelper = new CharacterizationHelper(
 *     "Elevator", elevator, elevator.getMotor());
 *
 * // Bind commands to buttons
 * controller.a().whileTrue(charHelper.quasistaticForward());
 * controller.b().whileTrue(charHelper.quasistaticReverse());
 * controller.x().whileTrue(charHelper.dynamicForward());
 * controller.y().whileTrue(charHelper.dynamicReverse());
 * }</pre>
 */
public class CharacterizationHelper {

    private final SysIdRoutine routine;

    /**
     * Create a characterization helper for a motor-driven mechanism.
     *
     * @param name descriptive name for logging
     * @param subsystem the subsystem that owns the motor (for command requirements)
     * @param motor the CatalystMotor to characterize
     */
    public CharacterizationHelper(String name, SubsystemBase subsystem, CatalystMotor motor) {
        this.routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(1.0), // ramp rate: 1 V/s
                        Volts.of(7.0),              // step voltage: 7V
                        Seconds.of(10.0)             // timeout: 10s
                ),
                new SysIdRoutine.Mechanism(
                        voltage -> motor.setVoltage(voltage.in(Volts)),
                        log -> {
                            log.motor(name)
                                    .voltage(Volts.of(motor.getAppliedVoltage()))
                                    .linearPosition(Meters.of(motor.getPosition()))
                                    .linearVelocity(MetersPerSecond.of(motor.getVelocity()));
                        },
                        subsystem
                )
        );
    }

    /**
     * Create a characterization helper for a rotational mechanism.
     * Logs position in rotations and velocity in rotations per second.
     *
     * @param name descriptive name for logging
     * @param subsystem the subsystem that owns the motor
     * @param motor the CatalystMotor to characterize
     * @param isRotational if true, log as angular units instead of linear
     */
    public CharacterizationHelper(String name, SubsystemBase subsystem, CatalystMotor motor,
                                   boolean isRotational) {
        if (!isRotational) {
            this.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(1.0),
                            Volts.of(7.0),
                            Seconds.of(10.0)
                    ),
                    new SysIdRoutine.Mechanism(
                            voltage -> motor.setVoltage(voltage.in(Volts)),
                            log -> {
                                log.motor(name)
                                        .voltage(Volts.of(motor.getAppliedVoltage()))
                                        .linearPosition(Meters.of(motor.getPosition()))
                                        .linearVelocity(MetersPerSecond.of(motor.getVelocity()));
                            },
                            subsystem
                    )
            );
        } else {
            this.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(1.0),
                            Volts.of(7.0),
                            Seconds.of(10.0)
                    ),
                    new SysIdRoutine.Mechanism(
                            voltage -> motor.setVoltage(voltage.in(Volts)),
                            log -> {
                                log.motor(name)
                                        .voltage(Volts.of(motor.getAppliedVoltage()))
                                        .angularPosition(Rotations.of(motor.getPosition()))
                                        .angularVelocity(RotationsPerSecond.of(motor.getVelocity()));
                            },
                            subsystem
                    )
            );
        }
    }

    /** Quasistatic forward test — slowly ramps voltage forward. */
    public Command quasistaticForward() {
        return routine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    /** Quasistatic reverse test — slowly ramps voltage backward. */
    public Command quasistaticReverse() {
        return routine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    /** Dynamic forward test — applies step voltage forward. */
    public Command dynamicForward() {
        return routine.dynamic(SysIdRoutine.Direction.kForward);
    }

    /** Dynamic reverse test — applies step voltage backward. */
    public Command dynamicReverse() {
        return routine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    /** Get the underlying SysIdRoutine for advanced use. */
    public SysIdRoutine getRoutine() {
        return routine;
    }
}
