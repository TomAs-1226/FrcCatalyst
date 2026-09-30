package frc.lib.catalyst.fieldlab;

import frc.lib.catalyst.util.MiniJson;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * What the campaign has already banked, on disk, so an hour of field time survives a power cycle.
 *
 * <h2>Why this exists</h2>
 *
 * <p>A campaign runs for an hour and the robot will be disabled, re-enabled, browned out and
 * power-cycled several times in that hour — a battery swap alone does all four. Without a checkpoint
 * every one of those restarts the campaign from the top, which means in practice the campaign never
 * gets past its third procedure and the operator gives up and runs things by hand. With one, a
 * restart costs the procedure that was in flight and nothing else.
 *
 * <p>Written after each procedure finishes, and re-read on start. It holds the session id, so a
 * resumed campaign keeps logging into the same namespace, and every measurement a person typed, so
 * nobody is asked to go and re-measure something they already measured.
 *
 * <h2>Where it goes</h2>
 *
 * <p>{@code /home/systemcore/catalyst/fieldlab-checkpoint.json} on a Systemcore, beside the motor
 * history, and in the working directory in simulation. Not on the USB stick the logs go to: the stick
 * gets pulled and taken to the shop, and a checkpoint that walks away mid-session is worse than none
 * because the campaign then silently restarts.
 *
 * <h2>Writing</h2>
 *
 * <p>To a temp file, then an atomic move. A checkpoint is written while the robot is running, and a
 * half-written JSON file read on the next boot would abort the resume and cost the whole session.
 * This is the same mistake motor history made once, where two writer threads shared one temp path;
 * here the temp name carries the process's own object identity so two runners cannot collide.
 *
 * @since 2.0.0
 */
public final class CampaignCheckpoint {

    /** Where a Systemcore keeps Catalyst's own state. */
    private static final Path SYSTEMCORE_DIR = Path.of("/home/systemcore/catalyst");

    private static final String FILE_NAME = "fieldlab-checkpoint.json";

    private final Path file;

    private String sessionId = "";
    private String campaignName = "";
    private final List<String> completed = new ArrayList<>();
    private final Map<String, Double> measurements = new LinkedHashMap<>();
    private final List<String> notes = new ArrayList<>();

    /** A checkpoint at the default location for this machine. */
    public CampaignCheckpoint() {
        this(defaultPath());
    }

    /** A checkpoint at an explicit path. Used by the tests, and by anyone who keeps state elsewhere. */
    public CampaignCheckpoint(Path file) {
        this.file = file;
    }

    /**
     * {@code /home/systemcore/catalyst/} when that directory exists, else the working directory.
     *
     * <p>Probed rather than chosen by {@code RobotBase.isSimulation()}, because a desktop test and a
     * simulation are not the same thing and neither has the Systemcore directory; what actually
     * matters is whether the directory is there.
     */
    private static Path defaultPath() {
        if (Files.isDirectory(SYSTEMCORE_DIR)) {
            return SYSTEMCORE_DIR.resolve(FILE_NAME);
        }
        return Path.of(FILE_NAME);
    }

    /** The session this checkpoint belongs to, or empty if nothing has been banked. */
    public String sessionId() {
        return sessionId;
    }

    /** The campaign name the checkpoint was written for. */
    public String campaignName() {
        return campaignName;
    }

    /** The ids of the procedures already finished, in the order they finished. */
    public List<String> completed() {
        return List.copyOf(completed);
    }

    /** Whether this procedure is already banked and should be skipped on a resume. */
    public boolean isComplete(String procedureId) {
        return completed.contains(procedureId);
    }

    /** Every measurement a person has typed, keyed {@code <procedure id>/<step key>}. */
    public Map<String, Double> measurements() {
        return Map.copyOf(measurements);
    }

    /** One measurement, if it has been taken. */
    public Optional<Double> measurement(String procedureId, String key) {
        return Optional.ofNullable(measurements.get(procedureId + "/" + key));
    }

    /** Free-text notes the runner recorded — timeouts, implausible values, gates that fired. */
    public List<String> notes() {
        return List.copyOf(notes);
    }

    /** Begin a session, discarding anything banked under a different campaign or session. */
    public void begin(String sessionId, String campaignName) {
        this.sessionId = sessionId;
        this.campaignName = campaignName;
        completed.clear();
        measurements.clear();
        notes.clear();
    }

    /** Record that a procedure finished. */
    public void complete(String procedureId) {
        if (!completed.contains(procedureId)) {
            completed.add(procedureId);
        }
    }

    /** Record a measurement a person typed. */
    public void measured(String procedureId, String key, double value) {
        measurements.put(procedureId + "/" + key, value);
    }

    /** Record a note. */
    public void note(String note) {
        notes.add(note);
    }

    /**
     * Read the checkpoint from disk.
     *
     * @return whether one was found and parsed. A missing or unreadable file is not an error — it
     *     means a fresh session, which is the common case — but a corrupt one is worth a note, so the
     *     reason is returned rather than swallowed.
     */
    public Optional<String> load() {
        if (!Files.isReadable(file)) {
            return Optional.of("no checkpoint at " + file + "; starting a fresh session");
        }
        try {
            String text = Files.readString(file, StandardCharsets.UTF_8);
            Map<String, Object> root = MiniJson.readObject(text);
            sessionId = str(root.get("session"));
            campaignName = str(root.get("campaign"));

            completed.clear();
            if (root.get("completed") instanceof List<?> list) {
                for (Object o : list) {
                    completed.add(str(o));
                }
            }

            measurements.clear();
            if (root.get("measurements") instanceof Map<?, ?> map) {
                for (Map.Entry<?, ?> e : map.entrySet()) {
                    if (e.getValue() instanceof Number n) {
                        measurements.put(str(e.getKey()), n.doubleValue());
                    }
                }
            }

            notes.clear();
            if (root.get("notes") instanceof List<?> list) {
                for (Object o : list) {
                    notes.add(str(o));
                }
            }
            return Optional.empty();
        } catch (IOException | RuntimeException e) {
            // Deliberately broad. A campaign that refuses to start because its checkpoint is
            // unreadable has turned a lost resume into a lost session.
            begin("", "");
            return Optional.of("checkpoint at " + file + " could not be read (" + e + "); starting a fresh session");
        }
    }

    /**
     * Write the checkpoint to disk, atomically.
     *
     * @return the reason it failed, or empty on success. A failed write is reported rather than
     *     thrown: it costs the resume, not the run in progress.
     */
    public Optional<String> save() {
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("session", sessionId);
        root.put("campaign", campaignName);
        root.put("completed", List.copyOf(completed));
        root.put("measurements", Map.copyOf(measurements));
        root.put("notes", List.copyOf(notes));

        Path parent = file.toAbsolutePath().getParent();
        Path temp = null;
        try {
            if (parent != null) {
                Files.createDirectories(parent);
            }
            // The temp name carries this object's identity so two runners in one JVM - which should
            // not happen, but did happen to motor history - cannot write the same temp file.
            temp = file.resolveSibling(file.getFileName() + "." + Integer.toHexString(System.identityHashCode(this)) + ".tmp");
            Files.writeString(temp, MiniJson.pretty(root), StandardCharsets.UTF_8);
            try {
                Files.move(temp, file, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
            } catch (AtomicMoveNotSupportedException e) {
                Files.move(temp, file, StandardCopyOption.REPLACE_EXISTING);
            }
            return Optional.empty();
        } catch (IOException | RuntimeException e) {
            if (temp != null) {
                try {
                    Files.deleteIfExists(temp);
                } catch (IOException ignored) {
                    // Nothing useful to do; a stray temp file is not worth failing over.
                }
            }
            return Optional.of("could not write the checkpoint to " + file + " (" + e + "); a restart will lose this session's progress");
        }
    }

    /** Delete the checkpoint. Called when a campaign finishes, so the next run starts clean. */
    public Optional<String> clear() {
        try {
            Files.deleteIfExists(file);
            return Optional.empty();
        } catch (IOException e) {
            return Optional.of("could not delete the checkpoint at " + file + " (" + e + ")");
        }
    }

    /** Where this checkpoint lives. */
    public Path path() {
        return file;
    }

    private static String str(Object o) {
        return o == null ? "" : String.valueOf(o);
    }
}
