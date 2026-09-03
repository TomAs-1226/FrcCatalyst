package frc.lib.catalyst.physics;

import frc.lib.catalyst.physics.data.SignalBuffer;
import frc.lib.catalyst.physics.data.TimestampSynchronizer;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * What a buffer does when a sensor hands it something that is not a number.
 *
 * <p>Every guard in {@link SignalBuffer} is a comparison, and NaN compares false to every operator,
 * so before this each guard was one bad reading away from being switched off permanently. The worst
 * of the four, measured on a copy of the class: after a single {@code add(+Infinity, ...)}, zero of
 * ten thousand following good samples were stored, and {@code latest()} kept answering the pre-fault
 * value as though it were current.
 *
 * <p>Every test here was checked against the unfixed class. The traps that make this kind of test
 * vacuous were checked for too: {@code sampleAt(±Infinity)} is deliberately not asserted, because
 * the existing range check already returns empty for both and such an assertion passes with the bug
 * present.
 */
class SignalBufferNonFiniteTest {

    // --- the value half ------------------------------------------------------

    @Test
    void aNaNValueIsRefusedRatherThanStored() {
        SignalBuffer b = new SignalBuffer(10);
        assertTrue(b.add(1.0, 10.0));
        assertTrue(b.add(2.0, 20.0));

        assertFalse(b.add(3.0, Double.NaN), "a reading that is not a number is not a reading");
        assertEquals(2, b.size(), "and it must not take a slot in the ring");
        assertEquals(20.0, b.latest(), 0.0, "latest() still answers the last real reading");
    }

    @Test
    void anInfiniteValueIsRefusedToo() {
        // Separate from the NaN case on purpose: a guard written as `Double.isNaN(value)` passes the
        // test above and still lets an infinity through, and an infinite value poisons sampleAt even
        // when it is asked for an exactly-stored timestamp.
        SignalBuffer b = new SignalBuffer(10);
        b.add(1.0, 5.0);

        assertFalse(b.add(2.0, Double.POSITIVE_INFINITY));
        assertEquals(1, b.size());
        assertEquals(5.0, b.sampleAt(1.0).orElseThrow(), 1e-9,
                "an exact hit on a stored sample must not come back NaN");
    }

    // --- the timestamp half, which is the dangerous one ----------------------

    @Test
    void aNaNTimestampDoesNotDisableTheOrderingGuardForever() {
        // The mechanism: NaN passes `timestampSeconds < latestTimestamp()`, and once stored,
        // latestTimestamp() IS NaN - so that comparison is false for every later sample too, and
        // out-of-order rejection never fires again.
        SignalBuffer b = new SignalBuffer(10);
        b.add(1.0, 10.0);
        b.add(2.0, 20.0);

        assertFalse(b.add(Double.NaN, 30.0));
        assertEquals(2.0, b.latestTimestamp(), 1e-9, "the ring's ordering is still intact");

        assertFalse(b.add(0.5, 5.0), "so a genuinely backwards sample is still rejected");
        assertEquals(2, b.size());
    }

    @Test
    void anInfiniteTimestampDoesNotWedgeTheBufferForever() {
        // The match-losing one. +Infinity is genuinely the largest value, so once it is the newest
        // timestamp every real sample afterwards is "backwards" and silently dropped - and the
        // buffer goes on reporting the pre-fault reading as live.
        SignalBuffer b = new SignalBuffer(10);
        b.add(1.0, 10.0);

        assertFalse(b.add(Double.POSITIVE_INFINITY, 99.0));

        for (int i = 2; i < 12; i++) {
            assertTrue(b.add(i, i * 10.0), "good sample at t=" + i + " was refused");
        }
        assertEquals(110.0, b.latest(), 1e-9, "the newest real reading, not a frozen one");
        assertEquals(11.0, b.latestTimestamp(), 1e-9);
    }

    @Test
    void aFaultLongerThanTheRingLeavesRealDataAndAStaleTimestamp() {
        // The regime a reviewer flagged: a sensor stuck non-finite for longer than the ring holds.
        // Nothing new can be stored, so the buffer keeps its last real samples. That is the intended
        // outcome - what is in the ring is real - and the staleness is visible because
        // latestTimestamp() stops advancing. A caller that cares about age has to look at it; a
        // buffer full of NaN would not have been better, only differently wrong.
        SignalBuffer b = new SignalBuffer(4);
        for (int i = 1; i <= 4; i++) {
            b.add(i, i * 10.0);
        }
        for (int i = 0; i < 50; i++) {
            assertFalse(b.add(Double.NaN, Double.NaN));
        }

        assertEquals(4, b.size());
        assertEquals(40.0, b.latest(), 1e-9);
        assertEquals(4.0, b.latestTimestamp(), 1e-9,
                "the timestamp stops advancing, which is how a caller sees the signal has gone quiet");
    }

    // --- reading it back -----------------------------------------------------

    @Test
    void askingForANaNTimeAnswersEmptyRatherThanNaN() {
        // NaN fails both halves of the range check, so without a guard it walks past "outside the
        // range" and comes back interpolated into NaN, presented as a reading that exists. The usual
        // source is a caller computing `frameTime - latency` with one of them not a number.
        SignalBuffer b = new SignalBuffer(10);
        b.add(1.0, 10.0);
        b.add(2.0, 20.0);

        assertTrue(b.sampleAt(Double.NaN).isEmpty());
    }

    @Test
    void ordinaryInterpolationIsUntouched() {
        // The guards must not have cost the thing the class is for.
        SignalBuffer b = new SignalBuffer(10);
        b.add(1.0, 10.0);
        b.add(2.0, 20.0);

        assertEquals(15.0, b.sampleAt(1.5).orElseThrow(), 1e-9);
        assertEquals(10.0, b.averageRate().orElseThrow(), 1e-9);
    }

    // --- the upstream boundary ----------------------------------------------

    @Test
    void aNonFiniteLatencyIsRefusedAtRegistration() {
        // The likeliest real route to a bad timestamp: latency is usually computed, often by a
        // division, and a zero frame count gives infinity. The old guard was `latencySeconds < 0`,
        // which is false for both NaN and +Infinity - and latency is subtracted from every capture
        // time this signal ever records, so one bad value here poisons the whole buffer rather than
        // a single sample.
        TimestampSynchronizer sync = new TimestampSynchronizer(10);

        assertThrows(IllegalArgumentException.class, () -> sync.register("vision", Double.NaN));
        assertThrows(IllegalArgumentException.class,
                () -> sync.register("vision", Double.POSITIVE_INFINITY));

        sync.register("vision", 0.02);   // and a real latency still registers
    }
}
