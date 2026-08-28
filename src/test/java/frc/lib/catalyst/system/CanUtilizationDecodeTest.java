package frc.lib.catalyst.system;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * What {@code /diagnostics/canbusutil} actually contains, and what it used to be read as.
 *
 * <p>Measured on a Systemcore running OS beta 13, one Pigeon on {@code can_s0}, the other four buses
 * idle. The topic published ten numbers for five buses:
 *
 * <pre>[5.341, 0.05341, 0, 0, 0, 0, 0, 0, 0, 0]</pre>
 *
 * <p>The second entry is exactly the first divided by a hundred, so these are
 * {@code [percent, fraction]} pairs. The agent's own frame counters agreed independently: 427
 * frames/s on a 1 Mbit bus is 4.6-5.8% depending on frame size, which makes 5.341 a percentage.
 *
 * <p>Reading it one-value-per-bus, which this class did, was wrong twice. {@code can_s0} came back
 * as 5.341 against a documented 0-1, and a dashboard multiplying by a hundred drew "534%".
 * {@code can_s1} came back as 0.05341 - bus zero's fraction wearing bus one's name, on a bus with no
 * traffic at all. That one is the more dangerous: 5% on an idle bus looks entirely reasonable.
 */
class CanUtilizationDecodeTest {

    @AfterEach
    void reset() {
        SystemCoreStatus.useSource(SystemCoreSource.unavailable());
    }

    /** A source that returns one fixed array for the diagnostics topic. */
    private static void publishing(double... raw) {
        SystemCoreStatus.useSource(new SystemCoreSource() {
            @Override public boolean isAvailable() { return true; }

            @Override public java.util.OptionalDouble number(String key) {
                return java.util.OptionalDouble.empty();
            }

            @Override public boolean bool(String key) { return false; }

            @Override public java.util.Optional<double[]> numberArray(String key) {
                return "/diagnostics/canbusutil".equals(key)
                        ? java.util.Optional.of(raw)
                        : java.util.Optional.empty();
            }
        });
    }

    @Test
    void theBoardsRealReadingBecomesAFractionPerBus() {
        publishing(5.341150038166781, 0.0534115003816678, 0, 0, 0, 0, 0, 0, 0, 0);

        double[] util = SystemCoreStatus.getInstance().canBusUtilization().orElseThrow();

        assertEquals(5, util.length, "five buses, whatever the topic's own shape is");
        assertEquals(0.05341, util[0], 1e-5,
                "5.341 percent is 0.05341 as the fraction this class documents");
        assertFalse(SystemCoreStatus.getInstance().canBusUtilizationOutOfRange());
    }

    @Test
    void anIdleBusReadsIdleRatherThanBorrowingTheBusyOnesFraction() {
        // The bug that mattered. can_s1 has no traffic; the old indexing gave it 0.05341, which
        // renders as a perfectly believable 5%.
        publishing(5.341150038166781, 0.0534115003816678, 0, 0, 0, 0, 0, 0, 0, 0);

        double[] util = SystemCoreStatus.getInstance().canBusUtilization().orElseThrow();

        assertEquals(0.0, util[1], 1e-12, "can_s1 is carrying nothing");
        for (int bus = 1; bus < util.length; bus++) {
            assertEquals(0.0, util[bus], 1e-12, "bus " + bus + " is idle");
        }
    }

    @Test
    void nothingIsScaledPastAHundredPercent() {
        // Every bus saturated: 100 percent each, which is 1.0 as a fraction and not 100.0.
        publishing(100, 1, 100, 1, 100, 1, 100, 1, 100, 1);

        double[] util = SystemCoreStatus.getInstance().canBusUtilization().orElseThrow();

        for (double v : util) {
            assertEquals(1.0, v, 1e-12);
        }
    }

    @Test
    void anUnexpectedShapeIsFlaggedRatherThanIndexedInto() {
        // The interleaving was read off a board with four buses idle, so it is consistent with the
        // evidence rather than proven by it. If the array stops being twice the bus count, the
        // assumption has expired and saying so beats guessing.
        publishing(0.05, 0.01, 0.0, 0.0, 0.0);

        double[] util = SystemCoreStatus.getInstance().canBusUtilization().orElseThrow();

        assertEquals(5, util.length, "handed back untouched");
        assertTrue(SystemCoreStatus.getInstance().canBusUtilizationOutOfRange(),
                "a shape nobody has measured must not be read as if it had been");
    }
}
