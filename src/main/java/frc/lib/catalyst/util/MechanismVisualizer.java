package frc.lib.catalyst.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Helper for creating Mechanism2d visualizations of robot mechanisms.
 * Mechanism2d is WPILib's built-in 2D visualization system that renders
 * in Shuffleboard, AdvantageScope, and Glass.
 *
 * <p>Example usage:
 * <pre>{@code
 * MechanismVisualizer viz = new MechanismVisualizer("Superstructure", 3, 3);
 *
 * // Create an elevator visualization
 * var elevatorViz = viz.addElevator("Elevator", 0.5, 0.5, 1.2, Color.kBlue);
 *
 * // Create an arm on top of the elevator
 * var armViz = viz.addArm("Arm", elevatorViz, 0.5, Color.kRed);
 *
 * // In periodic:
 * elevatorViz.setLength(elevator.getPosition());
 * armViz.setAngle(arm.getAngle());
 * }</pre>
 */
public class MechanismVisualizer {

    private final Mechanism2d mechanism;
    private final String name;

    /**
     * Create a Mechanism2d canvas.
     * @param name name for SmartDashboard/NetworkTables
     * @param width canvas width in meters
     * @param height canvas height in meters
     */
    public MechanismVisualizer(String name, double width, double height) {
        this.name = name;
        this.mechanism = new Mechanism2d(width, height);
        SmartDashboard.putData("Catalyst/" + name, mechanism);
    }

    /**
     * Add an elevator (vertical linear mechanism) visualization.
     * @param name ligament name
     * @param rootX x position of the base on the canvas (meters)
     * @param rootY y position of the base on the canvas (meters)
     * @param maxHeight maximum height for visual scaling
     * @param color line color
     * @return the MechanismLigament2d for updating in periodic
     */
    public MechanismLigament2d addElevator(String name, double rootX, double rootY,
                                            double maxHeight, Color color) {
        MechanismRoot2d root = mechanism.getRoot(name + "Root", rootX, rootY);
        MechanismLigament2d ligament = root.append(
                new MechanismLigament2d(name, 0, 90, 6, new Color8Bit(color)));
        return ligament;
    }

    /**
     * Add an arm (rotational mechanism) visualization at a fixed root.
     * @param name ligament name
     * @param rootX x position of the pivot (meters)
     * @param rootY y position of the pivot (meters)
     * @param length arm length in meters
     * @param color line color
     * @return the MechanismLigament2d for updating in periodic
     */
    public MechanismLigament2d addArm(String name, double rootX, double rootY,
                                       double length, Color color) {
        MechanismRoot2d root = mechanism.getRoot(name + "Root", rootX, rootY);
        MechanismLigament2d ligament = root.append(
                new MechanismLigament2d(name, length, 0, 6, new Color8Bit(color)));
        return ligament;
    }

    /**
     * Add an arm attached to the end of another ligament (e.g., arm on elevator).
     * @param name ligament name
     * @param parent the parent ligament to attach to
     * @param length arm length in meters
     * @param color line color
     * @return the MechanismLigament2d for updating in periodic
     */
    public MechanismLigament2d addArm(String name, MechanismLigament2d parent,
                                       double length, Color color) {
        return parent.append(
                new MechanismLigament2d(name, length, 0, 4, new Color8Bit(color)));
    }

    /**
     * Add a fixed structural element (doesn't move, just for reference).
     * @param name ligament name
     * @param rootX x position
     * @param rootY y position
     * @param length length in meters
     * @param angleDeg angle in degrees
     * @param color line color
     * @return the MechanismLigament2d
     */
    public MechanismLigament2d addFixed(String name, double rootX, double rootY,
                                         double length, double angleDeg, Color color) {
        MechanismRoot2d root = mechanism.getRoot(name + "Root", rootX, rootY);
        return root.append(
                new MechanismLigament2d(name, length, angleDeg, 4, new Color8Bit(color)));
    }

    /** Get the underlying Mechanism2d object. */
    public Mechanism2d getMechanism() {
        return mechanism;
    }
}
