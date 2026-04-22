package frc.lib.catalyst.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Tracks game piece state through a multi-stage mechanism pipeline.
 *
 * <p>Most robots move game pieces through several stages (intake → indexer →
 * shooter). This class provides a centralized state machine so every part
 * of the robot code can check piece status without passing booleans around.
 *
 * <p>Each stage is defined by a sensor (beam break, stall detection, etc.)
 * and the tracker automatically manages transitions. Publishes live state
 * to NetworkTables for dashboard visibility.
 *
 * <p>Example:
 * <pre>{@code
 * GamePieceTracker tracker = new GamePieceTracker("Piece")
 *     .addStage("Intake", intake::hasPiece)
 *     .addStage("Indexer", indexer::hasPiece)
 *     .addStage("Shooter", () -> shooterBeamBreak.get());
 *
 * // In periodic:
 * tracker.update();
 *
 * // Query state anywhere
 * boolean hasPiece = tracker.hasPiece();          // true if in ANY stage
 * boolean readyToShoot = tracker.isAt("Shooter"); // true if at shooter
 * int count = tracker.getPieceCount();             // total across all stages
 *
 * // Use as a Trigger for command bindings
 * tracker.atTrigger("Shooter").onTrue(leds.solid(Color.kGreen));
 * tracker.hasPieceTrigger().whileTrue(leds.blink(Color.kOrange, 5));
 * }</pre>
 */
public class GamePieceTracker {

    private final String name;
    private final Map<String, BooleanSupplier> stages = new LinkedHashMap<>();
    private final Map<String, Boolean> stageStates = new LinkedHashMap<>();
    private final NetworkTable table;

    /**
     * Create a game piece tracker.
     *
     * @param name tracker name (used for NetworkTables key, e.g. "Note", "Coral")
     */
    public GamePieceTracker(String name) {
        this.name = name;
        this.table = NetworkTableInstance.getDefault()
                .getTable("Catalyst").getSubTable("GamePiece").getSubTable(name);
    }

    /**
     * Add a stage to the pipeline.
     * Stages are evaluated in the order they are added.
     *
     * @param stageName display name (e.g., "Intake", "Indexer", "Shooter")
     * @param sensor sensor that detects a piece at this stage
     * @return this (for chaining)
     */
    public GamePieceTracker addStage(String stageName, BooleanSupplier sensor) {
        stages.put(stageName, sensor);
        stageStates.put(stageName, false);
        return this;
    }

    /**
     * Update all stage states. Call this once per periodic cycle.
     */
    public void update() {
        for (var entry : stages.entrySet()) {
            boolean state = entry.getValue().getAsBoolean();
            stageStates.put(entry.getKey(), state);
            table.getEntry(entry.getKey()).setBoolean(state);
        }
        table.getEntry("HasPiece").setBoolean(hasPiece());
        table.getEntry("PieceCount").setDouble(getPieceCount());
        table.getEntry("CurrentStage").setString(getCurrentStage());
    }

    /** Check if a game piece is detected at any stage. */
    public boolean hasPiece() {
        for (boolean state : stageStates.values()) {
            if (state) return true;
        }
        return false;
    }

    /** Check if a game piece is at a specific stage. */
    public boolean isAt(String stageName) {
        Boolean state = stageStates.get(stageName);
        return state != null && state;
    }

    /** Get the number of stages that currently detect a piece. */
    public int getPieceCount() {
        int count = 0;
        for (boolean state : stageStates.values()) {
            if (state) count++;
        }
        return count;
    }

    /**
     * Get the name of the furthest stage that has a piece.
     * Returns "None" if no piece detected.
     */
    public String getCurrentStage() {
        String last = "None";
        for (var entry : stageStates.entrySet()) {
            if (entry.getValue()) last = entry.getKey();
        }
        return last;
    }

    /**
     * Check if a piece is at the first stage (just picked up).
     */
    public boolean isAtFirst() {
        for (boolean state : stageStates.values()) {
            return state; // return the first stage's value
        }
        return false;
    }

    /**
     * Check if a piece is at the last stage (ready to score).
     */
    public boolean isAtLast() {
        boolean last = false;
        for (boolean state : stageStates.values()) {
            last = state;
        }
        return last;
    }

    /**
     * Create a Trigger that fires when a piece is at a specific stage.
     *
     * @param stageName stage to check
     * @return trigger that is true when piece is at the stage
     */
    public Trigger atTrigger(String stageName) {
        return new Trigger(() -> isAt(stageName));
    }

    /** Create a Trigger that fires when any piece is detected. */
    public Trigger hasPieceTrigger() {
        return new Trigger(this::hasPiece);
    }

    /** Create a Trigger that fires when no piece is detected. */
    public Trigger noPieceTrigger() {
        return new Trigger(() -> !hasPiece());
    }

    /** Create a Trigger that fires when the piece is at the last stage. */
    public Trigger readyToScoreTrigger() {
        return new Trigger(this::isAtLast);
    }

    /** Get the tracker name. */
    public String getName() {
        return name;
    }
}
