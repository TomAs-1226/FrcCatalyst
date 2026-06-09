package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Publishes simulated game-piece positions to NetworkTables so you can see
 * them on the field in AdvantageScope — the visualization half of physics
 * simulation, with no dependency on any particular sim engine.
 *
 * <p>This is dependency-free on purpose: it works whether you drive it from
 * <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/">maple-sim</a>,
 * a hand-rolled sim, or anything else. You compute the piece poses (maple-sim
 * gives them to you directly); this just streams them out as a
 * {@code Pose3d[]} that AdvantageScope renders.
 *
 * <pre>{@code
 * SimGamePieces fuel = new SimGamePieces("Fuel");
 *
 * @Override public void simulationPeriodic() {
 *     // e.g. from maple-sim's arena:
 *     for (var p : arena.getGamePiecesByType("Fuel")) {
 *         fuel.set(p.getId(), p.getPose3d());
 *     }
 *     fuel.publish();   // → /Catalyst/Sim/Fuel  (add as a Pose3d[] in AdvantageScope)
 * }
 * }</pre>
 *
 * <p>Add {@code /Catalyst/Sim/<name>} as a {@code Pose3d[]} source on your
 * AdvantageScope field/3D view to see the pieces.
 */
public final class SimGamePieces {

    private final Map<Object, Pose3d> pieces = new LinkedHashMap<>();
    private final StructArrayPublisher<Pose3d> pub;

    /**
     * @param name NT subtable name under {@code /Catalyst/Sim/} (e.g. "Fuel", "Coral")
     */
    public SimGamePieces(String name) {
        this.pub = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("Sim")
                .getStructArrayTopic(name, Pose3d.struct).publish();
    }

    /** Set (or update) one piece by id. */
    public void set(Object id, Pose3d pose) {
        if (pose != null) pieces.put(id, pose);
    }

    /** Remove a piece (e.g. it was intaked or scored). */
    public void remove(Object id) {
        pieces.remove(id);
    }

    /** Remove every piece. */
    public void clear() {
        pieces.clear();
    }

    /** Number of pieces currently on the field. */
    public int count() {
        return pieces.size();
    }

    /** Push the current set to NetworkTables. Call once per sim loop. */
    public void publish() {
        pub.set(pieces.values().toArray(new Pose3d[0]));
    }
}
