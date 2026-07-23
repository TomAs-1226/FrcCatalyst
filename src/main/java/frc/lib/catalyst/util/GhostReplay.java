package frc.lib.catalyst.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.catalyst.logging.CatalystLog;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Record a teleop drive and replay it later as a "ghost" pose for
 * training a new driver against a lead driver's path.
 *
 * <p>Two modes: {@link #startRecording(String)} captures the live pose
 * every loop into an in-memory buffer and writes it to disk when
 * recording stops. {@link #startReplay(String)} reads a saved trajectory
 * back and exposes the interpolated pose as a {@link Supplier} so a
 * dashboard (e.g. AdvantageScope) can render it on the field. The ghost
 * pose is also auto-published to {@code /Catalyst/Ghost/Pose}.
 *
 * <p>File format is a tiny CSV — {@code timestamp,x,y,theta} per line.
 * Files live under the deploy directory ({@code src/main/deploy/ghosts/}
 * during dev, {@code /home/lvuser/deploy/ghosts/} on the robot). Commit
 * the file to git and the ghost moves with the codebase.
 *
 * <p>Example wiring:
 * <pre>{@code
 * GhostReplay ghost = new GhostReplay(swerve::getPose);
 *
 * operator.start().onTrue(ghost.startRecording("lead-driver"));
 * operator.back() .onTrue(ghost.stopRecording());
 * operator.x()    .onTrue(ghost.startReplay("lead-driver"));
 * operator.b()    .onTrue(ghost.stopReplay());
 *
 * // In robotPeriodic, after the scheduler:
 * ghost.update();
 *
 * // Render the ghost in AdvantageScope by adding /Catalyst/Ghost/Pose
 * // as a Pose2d source on your field view.
 * }</pre>
 */
public final class GhostReplay {

    private static final String SUBDIR = "ghosts";
    private static final String EXT = ".csv";

    private final Supplier<Pose2d> liveSource;

    private final List<Sample> recordBuffer = new ArrayList<>();
    private boolean recording = false;
    private String activeRecordingName = "";
    private double recordStartTs = 0;

    private List<Sample> replayBuffer = new ArrayList<>();
    private boolean replaying = false;
    private double replayStartTs = 0;
    private Pose2d lastGhost = new Pose2d();

    /**
     * @param liveSource the robot's current pose. Typically
     *                   {@code swerve::getPose}.
     */
    public GhostReplay(Supplier<Pose2d> liveSource) {
        this.liveSource = liveSource;
    }

    // ----- Recording -----

    /** Command that begins recording under the given name. Idempotent. */
    public Command startRecording(String name) {
        return Commands.runOnce(() -> {
            recordBuffer.clear();
            activeRecordingName = name;
            recordStartTs = Timer.getFPGATimestamp();
            recording = true;
        }).ignoringDisable(true).withName("Ghost.StartRecording(" + name + ")");
    }

    /**
     * Command that stops recording and flushes the buffer to disk under
     * the most recently-active recording name.
     */
    public Command stopRecording() {
        return Commands.runOnce(() -> {
            if (!recording) return;
            recording = false;
            try {
                save(activeRecordingName, recordBuffer);
            } catch (IOException e) {
                System.err.println("[GhostReplay] failed to save \"" + activeRecordingName + "\": " + e.getMessage());
            }
        }).ignoringDisable(true).withName("Ghost.StopRecording");
    }

    /** True while a recording is in progress. */
    public boolean isRecording() { return recording; }

    /** Number of samples in the current recording buffer. */
    public int recordedSampleCount() { return recordBuffer.size(); }

    // ----- Replay -----

    /**
     * Command that loads a saved ghost from disk and begins playing it
     * back at its original timing. Reports a driver-station error and
     * does nothing if the file is missing.
     */
    public Command startReplay(String name) {
        return Commands.runOnce(() -> {
            try {
                replayBuffer = load(name);
                if (replayBuffer.isEmpty()) {
                    System.err.println("[GhostReplay] empty ghost \"" + name + "\"");
                    return;
                }
                replayStartTs = Timer.getFPGATimestamp();
                replaying = true;
                lastGhost = replayBuffer.get(0).pose();
            } catch (IOException e) {
                System.err.println("[GhostReplay] failed to load \"" + name + "\": " + e.getMessage());
            }
        }).ignoringDisable(true).withName("Ghost.StartReplay(" + name + ")");
    }

    /** Stop replay. The ghost pose freezes at its last value. */
    public Command stopReplay() {
        return Commands.runOnce(() -> replaying = false).ignoringDisable(true).withName("Ghost.StopReplay");
    }

    /** True while a replay is in progress. */
    public boolean isReplaying() { return replaying; }

    /**
     * The current ghost pose: the live one during recording, the
     * interpolated saved pose during replay, otherwise the last known
     * value.
     */
    public Pose2d currentGhostPose() { return lastGhost; }

    /** Supplier wrapper for the current ghost pose. */
    public Supplier<Pose2d> ghostSupplier() { return this::currentGhostPose; }

    // ----- Per-loop tick -----

    /**
     * Call once per loop from {@code Robot.robotPeriodic()} (after the
     * scheduler runs). Captures a sample when recording, advances the
     * replay clock, and publishes the ghost pose through CatalystLog.
     */
    public void update() {
        double now = Timer.getFPGATimestamp();
        if (recording) {
            double t = now - recordStartTs;
            recordBuffer.add(new Sample(t, liveSource.get()));
        }
        if (replaying && !replayBuffer.isEmpty()) {
            double t = now - replayStartTs;
            lastGhost = sampleAt(replayBuffer, t);
            if (t > replayBuffer.get(replayBuffer.size() - 1).t()) {
                // Looped past the end — leave the ghost on the final pose.
                replaying = false;
            }
        }
        CatalystLog.log("Ghost/Pose", Pose2d.struct, lastGhost);
    }

    // ----- Internals -----

    /** A timestamped pose. */
    public record Sample(double t, Pose2d pose) {}

    private static Pose2d sampleAt(List<Sample> samples, double t) {
        if (t <= samples.get(0).t()) return samples.get(0).pose();
        if (t >= samples.get(samples.size() - 1).t()) return samples.get(samples.size() - 1).pose();
        // Linear search is fine — buffers are typically a few thousand
        // samples and this is called once per loop.
        for (int i = 1; i < samples.size(); i++) {
            Sample a = samples.get(i - 1);
            Sample b = samples.get(i);
            if (t <= b.t()) {
                double alpha = (t - a.t()) / (b.t() - a.t());
                return interp(a.pose(), b.pose(), alpha);
            }
        }
        return samples.get(samples.size() - 1).pose();
    }

    private static Pose2d interp(Pose2d a, Pose2d b, double alpha) {
        Translation2d t = a.getTranslation().interpolate(b.getTranslation(), alpha);
        Rotation2d r = a.getRotation().interpolate(b.getRotation(), alpha);
        return new Pose2d(t, r);
    }

    private static File fileFor(String name) {
        File dir = new File(Filesystem.getDeployDirectory(), SUBDIR);
        if (!dir.exists()) dir.mkdirs();
        return new File(dir, name + EXT);
    }

    private static void save(String name, List<Sample> samples) throws IOException {
        File f = fileFor(name);
        try (PrintWriter pw = new PrintWriter(new FileWriter(f))) {
            pw.println("# Catalyst ghost replay — t,x,y,thetaDeg");
            for (Sample s : samples) {
                Pose2d p = s.pose();
                pw.printf("%.6f,%.4f,%.4f,%.3f%n",
                        s.t(), p.getX(), p.getY(), p.getRotation().getDegrees());
            }
        }
    }

    private static List<Sample> load(String name) throws IOException {
        File f = fileFor(name);
        if (!f.exists()) throw new IOException("ghost not found: " + f.getPath());
        List<Sample> out = new ArrayList<>();
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            String line;
            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                String[] parts = line.split(",");
                if (parts.length < 4) continue;
                double t = Double.parseDouble(parts[0]);
                double x = Double.parseDouble(parts[1]);
                double y = Double.parseDouble(parts[2]);
                double th = Double.parseDouble(parts[3]);
                out.add(new Sample(t, new Pose2d(x, y, Rotation2d.fromDegrees(th))));
            }
        }
        return out;
    }
}
