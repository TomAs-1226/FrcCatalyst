package frc.lib.catalyst.fieldlab;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The checkpoint is what lets an hour of field time survive the disables, brownouts and battery
 * swaps a real session puts it through. These exercise the disk round trip and its failure paths
 * directly, with an explicit {@code Path} under {@code @TempDir} so nothing touches
 * {@code /home/systemcore/catalyst} or the working directory.
 */
class CampaignCheckpointTest {

    @Test
    void aCheckpointWrittenAndReadBackHasTheSameSessionCompletedMeasurementsAndNotes(@TempDir Path dir) {
        Path file = dir.resolve("fieldlab-checkpoint.json");
        CampaignCheckpoint written = new CampaignCheckpoint(file);
        written.begin("abc12345", "Field Day");
        written.complete("wheel-radius");
        written.complete("slip-current");
        written.measured("wheel-radius", "caliper-diameter-in", 3.75);
        written.note("paused at 11.90 V, wanted 12.30 V");

        assertTrue(written.save().isEmpty(), "a clean write should not report a problem");

        CampaignCheckpoint reloaded = new CampaignCheckpoint(file);
        assertTrue(reloaded.load().isEmpty(), "a checkpoint that was just written should load without complaint");
        assertEquals("abc12345", reloaded.sessionId());
        assertEquals("Field Day", reloaded.campaignName());
        assertEquals(List.of("wheel-radius", "slip-current"), reloaded.completed());
        assertEquals(3.75, reloaded.measurement("wheel-radius", "caliper-diameter-in").orElseThrow(), 1e-9);
        assertEquals(List.of("paused at 11.90 V, wanted 12.30 V"), reloaded.notes());
    }

    @Test
    void textThatIsNotJsonAtAllLeavesTheCheckpointEmptyAndReportsAReasonInsteadOfThrowing(@TempDir Path dir)
            throws IOException {
        Path file = dir.resolve("fieldlab-checkpoint.json");
        Files.writeString(file, "not json at all {{{");

        CampaignCheckpoint cp = new CampaignCheckpoint(file);
        Optional<String> reason = cp.load();

        assertTrue(reason.isPresent(), "a corrupt checkpoint should be reported, not silently accepted");
        assertFalse(reason.get().isBlank(), "the reason should say something, not just be an empty string");
        assertEquals("", cp.sessionId(), "a corrupt read should leave the session empty rather than half-populated");
        assertEquals(List.of(), cp.completed());
        assertEquals(Map.of(), cp.measurements());
    }

    @Test
    void validJsonThatIsNotAnObjectIsTreatedAsCorruptTooNotAsACrash(@TempDir Path dir) throws IOException {
        Path file = dir.resolve("fieldlab-checkpoint.json");
        Files.writeString(file, "[1, 2, 3]");

        CampaignCheckpoint cp = new CampaignCheckpoint(file);
        Optional<String> reason = cp.load();

        assertTrue(reason.isPresent(), "valid JSON that is not the expected object shape should still be reported");
        assertFalse(reason.get().isBlank());
        assertEquals("", cp.sessionId());
        assertEquals(List.of(), cp.completed());
    }

    @Test
    void aMissingCheckpointIsNotAnErrorAndSaysSoInTheReason(@TempDir Path dir) {
        CampaignCheckpoint cp = new CampaignCheckpoint(dir.resolve("does-not-exist.json"));

        Optional<String> reason = cp.load();

        assertTrue(reason.isPresent(), "a missing checkpoint still explains why nothing loaded");
        assertTrue(reason.get().contains("no checkpoint"),
                "a missing file should read as 'no checkpoint', not as a failure: " + reason.get());
    }

    @Test
    void aSuccessfulSaveLeavesNoTmpFileBesideTheCheckpoint(@TempDir Path dir) throws IOException {
        Path file = dir.resolve("fieldlab-checkpoint.json");
        CampaignCheckpoint cp = new CampaignCheckpoint(file);
        cp.begin("s1", "Campaign");
        cp.complete("p1");

        assertTrue(cp.save().isEmpty());

        try (var listing = Files.list(dir)) {
            List<String> names = listing.map(p -> p.getFileName().toString()).toList();
            assertTrue(names.stream().noneMatch(n -> n.endsWith(".tmp")),
                    "a successful save should leave no temp file behind: " + names);
        }
    }

    @Test
    void clearDeletesTheCheckpointAndClearingAnAlreadyMissingOneIsNotAnError(@TempDir Path dir) throws IOException {
        Path file = dir.resolve("fieldlab-checkpoint.json");
        CampaignCheckpoint cp = new CampaignCheckpoint(file);
        cp.begin("s1", "Campaign");
        cp.save();
        assertTrue(Files.exists(file), "the checkpoint should exist before it is cleared");

        assertTrue(cp.clear().isEmpty());
        assertFalse(Files.exists(file), "clear should remove the file");

        assertTrue(cp.clear().isEmpty(), "clearing a checkpoint that is already gone should not be an error");
    }
}
