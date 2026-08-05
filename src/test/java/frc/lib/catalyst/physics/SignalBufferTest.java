package frc.lib.catalyst.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;

import frc.lib.catalyst.physics.data.SignalBuffer;

/** Pure-Java tests for {@link SignalBuffer}. No HAL, no NetworkTables, no robot. */
class SignalBufferTest {

    @Test
    void interpolatesBetweenStraddlingSamples() {
        SignalBuffer buffer = new SignalBuffer(10);
        buffer.add(1.0, 10.0);
        buffer.add(2.0, 20.0);

        assertEquals(10.0, buffer.sampleAt(1.0).getAsDouble(), 1e-9);
        assertEquals(15.0, buffer.sampleAt(1.5).getAsDouble(), 1e-9);
        assertEquals(20.0, buffer.sampleAt(2.0).getAsDouble(), 1e-9);
    }

    @Test
    void refusesToExtrapolateOutsideWhatItHolds() {
        SignalBuffer buffer = new SignalBuffer(10);
        buffer.add(1.0, 10.0);
        buffer.add(2.0, 20.0);

        assertTrue(buffer.sampleAt(0.5).isEmpty());   // before the oldest
        assertTrue(buffer.sampleAt(2.5).isEmpty());   // after the newest
    }

    @Test
    void emptyBufferAnswersNothing() {
        SignalBuffer buffer = new SignalBuffer(4);
        assertTrue(buffer.isEmpty());
        assertEquals(OptionalDouble.empty(), buffer.sampleAt(0.0));
        assertEquals(OptionalDouble.empty(), buffer.averageRate());
        assertEquals(0.0, buffer.latest(), 1e-9);
    }

    @Test
    void oldestSamplesFallOffTheRingAndLookupsStillWork() {
        SignalBuffer buffer = new SignalBuffer(3);
        for (int i = 0; i < 6; i++) buffer.add(i, i * 10.0);   // t=0..5, v=0..50

        assertEquals(3, buffer.size());
        assertEquals(3.0, buffer.oldestTimestamp(), 1e-9);      // t=0,1,2 evicted
        assertEquals(5.0, buffer.latestTimestamp(), 1e-9);
        assertEquals(45.0, buffer.sampleAt(4.5).getAsDouble(), 1e-9);
        assertTrue(buffer.sampleAt(2.0).isEmpty());             // evicted, so unknown
    }

    @Test
    void samplesGoingBackwardsInTimeAreDropped() {
        SignalBuffer buffer = new SignalBuffer(10);
        assertTrue(buffer.add(2.0, 20.0));
        assertFalse(buffer.add(1.0, 10.0));   // would break the ordering lookups rely on

        assertEquals(1, buffer.size());
        assertEquals(20.0, buffer.latest(), 1e-9);
    }

    @Test
    void averageRateIsTheChordSlopeAcrossTheBuffer() {
        SignalBuffer buffer = new SignalBuffer(10);
        buffer.add(0.0, 0.0);
        buffer.add(1.0, 5.0);
        buffer.add(2.0, 20.0);

        assertEquals(10.0, buffer.averageRate().getAsDouble(), 1e-9);   // (20 - 0) / 2
    }

    @Test
    void repeatedTimestampsDoNotDivideByZero() {
        SignalBuffer buffer = new SignalBuffer(10);
        buffer.add(1.0, 10.0);
        buffer.add(1.0, 12.0);

        assertEquals(12.0, buffer.sampleAt(1.0).getAsDouble(), 1e-9);
        assertTrue(buffer.averageRate().isEmpty());   // no time elapsed, no rate
    }

    @Test
    void clearReturnsItToEmpty() {
        SignalBuffer buffer = new SignalBuffer(4);
        buffer.add(1.0, 1.0);
        buffer.add(2.0, 2.0);
        buffer.clear();

        assertTrue(buffer.isEmpty());
        assertTrue(buffer.add(0.5, 5.0));   // and accepts an earlier timestamp again
    }

    @Test
    void capacityBelowTwoIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> new SignalBuffer(1));
        assertThrows(IllegalArgumentException.class, () -> new SignalBuffer(0));
    }
}
