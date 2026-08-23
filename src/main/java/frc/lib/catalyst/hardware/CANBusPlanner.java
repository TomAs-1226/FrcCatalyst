package frc.lib.catalyst.hardware;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Works out how to spread CAN devices across Systemcore's five buses.
 *
 * <p>One bus meant there was no decision to make. Five means there is, and it is not the obvious
 * one: the buses are <b>not</b> five independent lanes. Read out of the Systemcore OS image, they
 * are MCP2518FD controllers on shared SPI hosts — {@code can_s0}+{@code can_s1} on one,
 * {@code can_s3}+{@code can_s4} on another, {@code can_s2} alone. Two heavily loaded buses that
 * happen to be paired throttle each other; two unpaired ones do not. That is why the Systemcore web
 * UI warns about frame-rate limits "for any individual CAN Bus <i>or CAN Bus pair</i>", and it is
 * why splitting a drivetrain across {@code can_s0} and {@code can_s1} buys much less than splitting
 * it across {@code can_s0} and {@code can_s3}.
 *
 * <p>{@link #suggest()} does the packing with that constraint in it. {@link #validate()} checks a
 * plan that already exists.
 *
 * <h2>On the load model, and how much to trust it</h2>
 *
 * <p>This estimates bus load from device counts and per-device frame rates. Those rates are
 * <b>estimates</b>: Phoenix's default status-signal configuration is not something Catalyst can read
 * out of a jar, and it changes with firmware and with whatever a team has configured. The defaults
 * here are deliberately conservative — they will over-state load rather than under-state it, because
 * a planner that says "you are fine" and is wrong is worse than one that says "you are close" and is
 * wrong.
 *
 * <p>The honest way to use this is as the first half of a loop: plan with the estimate, then measure
 * with {@link CANBusHealth} on the real robot and feed the measured utilisation back in with
 * {@link #calibrate(String, double)}. An estimate that has been checked against a bus analyser beats
 * one that has not, and the estimate is only ever a starting point for where to put things.
 *
 * @since 2.0.0
 */
public final class CANBusPlanner {

    /**
     * Frames per second attributed to a device whose type is not recognised.
     *
     * <p>Chosen to sit near a Phoenix device left on its default status-signal rates. A device that
     * has had {@code optimizeCanUtilization} applied produces considerably less.
     */
    public static final double DEFAULT_FRAMES_PER_SECOND = 200.0;

    /**
     * Bits on the wire per CAN 2.0 frame, including stuffing and inter-frame space.
     *
     * <p>An 8-byte frame is 111 bits before stuffing; bit stuffing and the inter-frame gap push a
     * realistic worst case to roughly this. Systemcore runs its own buses at 1 Mbit/s with FD off,
     * so this is the right frame model for them.
     */
    private static final double BITS_PER_FRAME = 135.0;

    /** Bus bitrate. Systemcore brings every one of its buses up at 1 Mbit/s, CAN 2.0. */
    private static final double BUS_BITS_PER_SECOND = 1_000_000.0;

    /**
     * Utilisation a single bus should stay under.
     *
     * <p>Systemcore warns at 90%. Planning to 90% leaves nothing for the bursts that occur when
     * every mechanism is commanded at once, so the planner aims lower.
     */
    public static final double TARGET_BUS_UTILIZATION = 0.60;

    /**
     * Combined utilisation a paired bus should stay under.
     *
     * <p>Not simply twice the single-bus target: the pair shares one SPI host, so the ceiling is
     * about the controller's throughput rather than the sum of two independent wires.
     */
    public static final double TARGET_PAIR_UTILIZATION = 1.00;

    /**
     * Device count on one bus that earns advice to spread out, when other buses are free.
     *
     * <p>Independent of the load estimate on purpose — see {@link #validate()}. Ten is roughly a
     * swerve drivetrain plus its encoders, which is the point at which a second bus starts being
     * obviously worth using.
     */
    public static final int CONCENTRATION_ADVICE_THRESHOLD = 10;

    /** Status signals CatalystMotor raises on a primary motor when optimising. */
    private static final int PRIMARY_SIGNALS = 9;

    /** Status signals CatalystMotor raises on a follower when optimising. */
    private static final int FOLLOWER_SIGNALS = 2;

    /** Per-device-type frame rate estimates, keyed by the type string the registry stores. */
    private static final Map<String, Double> FRAME_RATES = new LinkedHashMap<>();

    /** Measured utilisation, when a team has supplied real numbers. */
    private static final Map<String, Double> CALIBRATED = new LinkedHashMap<>();

    /**
     * Devices whose status-signal rates were cut, and the rate they were cut to.
     *
     * <p>Populated by {@code CatalystMotor} when {@code optimizeCanBus(...)} is used. A device on
     * default Phoenix rates and one trimmed to 50 Hz differ by several times in bus cost, and a
     * planner that prices them the same will recommend rewiring a bus that was never the problem.
     */
    private static final Map<String, Double> OPTIMIZED = new LinkedHashMap<>();

    static {
        // Conservative estimates. A motor talks far more than a sensor: it publishes position,
        // velocity, current, voltage and fault signals continuously, where an encoder publishes
        // position and little else.
        FRAME_RATES.put("TalonFX", 250.0);
        FRAME_RATES.put("TalonFX (follower)", 150.0);
        FRAME_RATES.put("CANcoder", 100.0);
        FRAME_RATES.put("CANcoder (fused)", 100.0);
        FRAME_RATES.put("CANcoder (sync)", 100.0);
        FRAME_RATES.put("CANcoder (remote)", 100.0);
        FRAME_RATES.put("Pigeon2", 100.0);
    }

    private CANBusPlanner() {}

    /** Override the frame-rate estimate for a device type, e.g. after measuring one. */
    public static synchronized void setFrameRate(String deviceType, double framesPerSecond) {
        FRAME_RATES.put(deviceType, framesPerSecond);
    }

    /**
     * Supply a measured utilisation for a bus, overriding the estimate for it.
     *
     * <p>Read it off {@link CANBusHealth} on a real robot under load, not at rest — a bus at idle
     * tells you nothing about a bus during a match.
     *
     * @param busName      bus name, e.g. {@code "can_s0"}
     * @param utilization  measured utilisation, 0–1
     */
    public static synchronized void calibrate(String busName, double utilization) {
        CALIBRATED.put(CatalystCANBus.of(busName).name(), utilization);
    }

    /** Estimated frames per second for one device type, ignoring any per-device optimisation. */
    public static synchronized double framesPerSecond(String deviceType) {
        return FRAME_RATES.getOrDefault(deviceType, DEFAULT_FRAMES_PER_SECOND);
    }

    /**
     * Record that a device's status signals were trimmed to a fixed rate.
     *
     * <p>Called by {@code CatalystMotor.Builder.optimizeCanBus(...)}. Not something robot code needs
     * to call directly.
     *
     * @param deviceName registry name of the device
     * @param updateHz   rate its signals were set to
     */
    public static synchronized void noteOptimizedDevice(String deviceName, double updateHz) {
        OPTIMIZED.put(deviceName, updateHz);
    }

    /**
     * Frames per second for one specific registered device.
     *
     * <p>An optimised device is priced from the signals Catalyst actually raises rather than from
     * the type default. {@code CatalystMotor} raises nine signals on a primary and two on a
     * follower, so the cost is that count multiplied by the configured rate — which is both a
     * better estimate and one whose derivation is visible.
     */
    public static synchronized double framesPerSecondFor(CANRegistry.Entry device) {
        Double optimisedHz = OPTIMIZED.get(device.name());
        if (optimisedHz == null) {
            return framesPerSecond(device.type());
        }
        int signals = device.type().contains("follower") ? FOLLOWER_SIGNALS : PRIMARY_SIGNALS;
        return signals * optimisedHz;
    }

    /**
     * Predicted utilisation of a bus, 0–1.
     *
     * <p>Returns the measured figure when one has been supplied through
     * {@link #calibrate(String, double)}, because a measurement always beats a model.
     */
    public static synchronized double utilizationOf(String busName) {
        String name = CatalystCANBus.of(busName).name();
        Double measured = CALIBRATED.get(name);
        if (measured != null) {
            return measured;
        }
        double frames = 0;
        for (CANRegistry.Entry e : CANRegistry.byBus().getOrDefault(busName, List.of())) {
            frames += framesPerSecondFor(e);
        }
        return frames * BITS_PER_FRAME / BUS_BITS_PER_SECOND;
    }

    /**
     * Problems with the wiring plan as it currently stands.
     *
     * <p>Three kinds, in increasing order of how easy they are to miss:
     * a bus over its own target; an SPI pair over the pair target even though neither bus alone
     * looks bad; and buses sitting empty while another is overloaded.
     *
     * <p>Advisory only — nothing throws.
     */
    public static synchronized List<String> validate() {
        List<String> problems = new ArrayList<>();
        Map<String, List<CANRegistry.Entry>> byBus = CANRegistry.byBus();

        // Individual buses.
        for (String bus : byBus.keySet()) {
            double util = utilizationOf(bus);
            if (util > TARGET_BUS_UTILIZATION) {
                problems.add(String.format(
                        "%s is at an estimated %.0f%% utilisation (target %.0f%%). %d devices.",
                        CatalystCANBus.of(bus).name(), util * 100,
                        TARGET_BUS_UTILIZATION * 100, byBus.get(bus).size()));
            }
        }

        // Pairs. This is the one that hides: two buses at 55% each look fine individually and
        // together exceed what their shared SPI controller can carry.
        List<String> names = new ArrayList<>(byBus.keySet());
        for (int i = 0; i < names.size(); i++) {
            for (int j = i + 1; j < names.size(); j++) {
                CatalystCANBus a = safe(names.get(i));
                CatalystCANBus b = safe(names.get(j));
                if (a == null || b == null || !a.sharesControllerWith(b)) {
                    continue;
                }
                double combined = utilizationOf(names.get(i)) + utilizationOf(names.get(j));
                if (combined > TARGET_PAIR_UTILIZATION) {
                    problems.add(String.format(
                            "%s and %s share an SPI controller and together sit at an estimated "
                                    + "%.0f%%. Neither looks bad alone - move some devices onto an "
                                    + "unpaired bus rather than shuffling them between these two.",
                            a.name(), b.name(), combined * 100));
                }
            }
        }

        // Concentration, independent of the estimate.
        //
        // This is deliberately not gated behind the utilisation checks above. The load model is an
        // estimate and a conservative one; "twelve devices on one bus while three sit empty" is
        // worth saying regardless of what the model predicts, because the advice does not depend on
        // the number being right and the fix is nearly free.
        long busesUsed = byBus.size();
        if (busesUsed < CatalystCANBus.SYSTEMCORE_BUS_COUNT) {
            byBus.forEach((bus, devices) -> {
                if (devices.size() >= CONCENTRATION_ADVICE_THRESHOLD) {
                    problems.add(String.format(
                            "%s carries %d devices while %d of Systemcore's %d buses are unused. "
                                    + "Spreading them out costs nothing and beats tuning "
                                    + "status-signal rates later.",
                            CatalystCANBus.of(bus).name(), devices.size(),
                            CatalystCANBus.SYSTEMCORE_BUS_COUNT - busesUsed,
                            CatalystCANBus.SYSTEMCORE_BUS_COUNT));
                }
            });
        }
        return problems;
    }

    /**
     * A proposed assignment of the currently registered devices to buses.
     *
     * <p>Greedy, heaviest device first, each one placed on the bus whose <em>SPI group</em> is least
     * loaded — grouping rather than per-bus, because that is the constraint that actually binds.
     * {@code can_s2} is unpaired and therefore the most valuable place to put a heavy device, so it
     * wins ties.
     *
     * <p>This is advice, not an instruction. It knows nothing about where things physically sit on
     * the robot, and a plan that halves bus load while doubling the length of the CAN wiring is not
     * an improvement. Read it as "these devices want separating", then decide where.
     *
     * @return bus name to device names, for every bus that gets something
     */
    public static synchronized Map<String, List<String>> suggest() {
        List<CANRegistry.Entry> devices = new ArrayList<>(CANRegistry.all());
        devices.sort(Comparator.comparingDouble(
                CANBusPlanner::framesPerSecondFor).reversed());

        List<CatalystCANBus> buses = new ArrayList<>();
        for (int i = 0; i < CatalystCANBus.SYSTEMCORE_BUS_COUNT; i++) {
            buses.add(CatalystCANBus.systemcore(i));
        }

        Map<String, List<String>> plan = new LinkedHashMap<>();
        Map<String, Double> busFrames = new LinkedHashMap<>();
        Map<Integer, Double> groupFrames = new LinkedHashMap<>();
        for (CatalystCANBus b : buses) {
            busFrames.put(b.name(), 0.0);
            groupFrames.merge(b.controllerGroup(), 0.0, Double::sum);
        }

        for (CANRegistry.Entry device : devices) {
            double load = framesPerSecondFor(device);

            CatalystCANBus target = buses.get(0);
            double bestGroupLoad = Double.MAX_VALUE;
            for (CatalystCANBus b : buses) {
                double groupLoad = groupFrames.getOrDefault(b.controllerGroup(), 0.0);
                // Tie-break toward the unpaired bus: a device there costs nothing to any other bus.
                boolean better = groupLoad < bestGroupLoad
                        || (groupLoad == bestGroupLoad && isUnpaired(b) && !isUnpaired(target));
                if (better) {
                    bestGroupLoad = groupLoad;
                    target = b;
                }
            }

            plan.computeIfAbsent(target.name(), k -> new ArrayList<>()).add(device.name());
            busFrames.merge(target.name(), load, Double::sum);
            groupFrames.merge(target.controllerGroup(), load, Double::sum);
        }
        return plan;
    }

    /** Whether a bus has its SPI controller to itself — true only for {@code can_s2}. */
    private static boolean isUnpaired(CatalystCANBus bus) {
        int group = bus.controllerGroup();
        int sharing = 0;
        for (int i = 0; i < CatalystCANBus.SYSTEMCORE_BUS_COUNT; i++) {
            if (CatalystCANBus.systemcore(i).controllerGroup() == group) {
                sharing++;
            }
        }
        return sharing == 1;
    }

    private static CatalystCANBus safe(String bus) {
        try {
            return CatalystCANBus.of(bus);
        } catch (RuntimeException ignored) {
            return null;
        }
    }
}
