package frc.lib.catalyst.identity;

import frc.lib.catalyst.system.SystemCoreStatus;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * What this robot is made of, and how much of it is currently answering.
 *
 * <h2>The question it answers</h2>
 *
 * <p>The spec sheet ({@link RobotIdentity}) says what was <em>declared</em>: a motor at CAN id 12 on
 * {@code can_s0}, a camera called {@code limelight-left}. It is written once at boot and cannot say
 * whether the motor is on the bus or the camera has a NetworkTables session. The alert manager says
 * when something has gone <em>wrong</em>, one line per fault. Neither gives the glance a dashboard
 * corner needs - "4 of 4 cameras, 20 of 20 motors, controller up" - which is the first thing anyone
 * looks for when a robot behaves strangely, and is currently answered by walking round the robot.
 *
 * <p>So: three counts, published a few times a second, and a row per device behind each count. The
 * Catalyst hardware classes register themselves here as they are constructed, so a team that never
 * heard of this class still gets the counts.
 *
 * <h2>Connected means</h2>
 *
 * <ul>
 *   <li><b>Motor</b> - Phoenix reports the device answering on its bus.
 *   <li><b>Camera</b> - the source says so; for a Limelight, its heartbeat topic is still advancing.
 *       A source that cannot tell reports connected, and says so in its detail.
 *   <li><b>Controller</b> - the Systemcore system server is reachable. On anything else the robot
 *       program running is the only evidence there is, so it is reported connected with that caveat
 *       in the kind.
 * </ul>
 *
 * <h2>Keys</h2>
 *
 * <pre>
 *   /Catalyst/Devices/Cameras/Expected      int
 *   /Catalyst/Devices/Cameras/Connected     int
 *   /Catalyst/Devices/Cameras/Rows          string[]   "name|connected|detail"
 *   /Catalyst/Devices/Motors/Expected|Connected|Rows      "name|bus|id|connected"
 *   /Catalyst/Devices/Controller/Kind       string     "Systemcore" / "Robot controller"
 *   /Catalyst/Devices/Controller/Connected  bool
 *   /Catalyst/Devices/Summary               string     "4/4 cameras · 20/20 motors · Systemcore"
 * </pre>
 *
 * <p>Rows are pipe-delimited strings for the same reason {@code Hardware/Devices} is: one topic a
 * dashboard can read without knowing the device names in advance.
 *
 * <p>Published from {@code HealthMonitor.update()}, which {@code CatalystOpMode} drives whether the
 * robot is enabled or not, at most four times a second. Connectivity probes are cheap - Phoenix
 * answers from a cached timestamp, NetworkTables from a cached one too - but there is no reason to
 * ask fifty times a second for a number that changes once a match.
 *
 * @since 2.0.0
 */
public final class DeviceRoster {

    /** What a device is, for the count it belongs in. */
    public enum Kind { CAMERA, MOTOR, CONTROLLER }

    /**
     * One registered device.
     *
     * @param kind      which count it belongs in
     * @param name      the name the team gave it, or a generated one
     * @param detail    bus and id for a motor, API for a camera, model for a controller
     * @param connected answers whether it is currently reachable
     */
    public record Device(Kind kind, String name, String detail, BooleanSupplier connected) {
        /** Whether it is reachable right now. A probe that throws counts as not connected. */
        public boolean isConnected() {
            try {
                return connected.getAsBoolean();
            } catch (RuntimeException e) {
                return false;
            }
        }
    }

    /** One count's worth of devices, as published. */
    public record Count(int expected, int connected) {}

    private static final double PUBLISH_PERIOD_SECONDS = 0.25;

    private static final List<Device> devices = new ArrayList<>();
    private static Device controller;
    private static double lastPublishTs = Double.NaN;
    private static NetworkTable table;

    private DeviceRoster() {}

    /** Register any device. The Catalyst hardware classes do this themselves. */
    public static synchronized void register(Device device) {
        if (device.kind() == Kind.CONTROLLER) {
            controller = device;
            return;
        }
        devices.add(device);
    }

    /** Register a motor by what a dashboard needs to show about it. */
    public static void registerMotor(String name, String bus, int canId, BooleanSupplier connected) {
        register(new Device(Kind.MOTOR, name, bus + "|" + canId, connected));
    }

    /** Register a camera. {@code detail} is free text - the API it speaks, or the model. */
    public static void registerCamera(String name, String detail, BooleanSupplier connected) {
        register(new Device(Kind.CAMERA, name, detail, connected));
    }

    /** Register (or replace) the robot controller. There is only ever one. */
    public static void registerController(String kind, BooleanSupplier connected) {
        register(new Device(Kind.CONTROLLER, kind, kind, connected));
    }

    /** Every registered device except the controller, in registration order. */
    public static synchronized List<Device> devices() {
        return List.copyOf(devices);
    }

    /** Expected and connected, for one kind. Probes every device of that kind. */
    public static synchronized Count count(Kind kind) {
        int expected = 0;
        int connected = 0;
        for (Device d : devices) {
            if (d.kind() != kind) continue;
            expected++;
            if (d.isConnected()) connected++;
        }
        return new Count(expected, connected);
    }

    /**
     * Publish the counts and rows if a quarter second has passed since the last time.
     *
     * @param now seconds, monotonic - {@code Timer.getTimestamp()}
     */
    public static synchronized void publish(double now) {
        if (!Double.isNaN(lastPublishTs) && now - lastPublishTs < PUBLISH_PERIOD_SECONDS) return;
        lastPublishTs = now;
        publishNow();
    }

    /** Publish immediately, ignoring the rate limit. For tests and for a one-off refresh. */
    public static synchronized void publishNow() {
        if (table == null) {
            table = NetworkTableInstance.getDefault().getTable("Catalyst").getSubTable("Devices");
        }
        if (controller == null) {
            controller = defaultController();
        }

        List<String> cameraRows = new ArrayList<>();
        List<String> motorRows = new ArrayList<>();
        int camerasUp = 0;
        int motorsUp = 0;
        for (Device d : devices) {
            boolean up = d.isConnected();
            if (d.kind() == Kind.CAMERA) {
                if (up) camerasUp++;
                cameraRows.add(d.name() + "|" + up + "|" + d.detail());
            } else if (d.kind() == Kind.MOTOR) {
                if (up) motorsUp++;
                motorRows.add(d.name() + "|" + d.detail() + "|" + up);
            }
        }

        NetworkTable cams = table.getSubTable("Cameras");
        cams.getEntry("Expected").setInteger(cameraRows.size());
        cams.getEntry("Connected").setInteger(camerasUp);
        cams.getEntry("Rows").setStringArray(cameraRows.toArray(new String[0]));

        NetworkTable motors = table.getSubTable("Motors");
        motors.getEntry("Expected").setInteger(motorRows.size());
        motors.getEntry("Connected").setInteger(motorsUp);
        motors.getEntry("Rows").setStringArray(motorRows.toArray(new String[0]));

        boolean controllerUp = controller.isConnected();
        NetworkTable ctl = table.getSubTable("Controller");
        ctl.getEntry("Kind").setString(controller.name());
        ctl.getEntry("Connected").setBoolean(controllerUp);

        table.getEntry("Summary").setString(
                camerasUp + "/" + cameraRows.size() + " cameras · "
                + motorsUp + "/" + motorRows.size() + " motors · "
                + controller.name() + (controllerUp ? "" : " (not answering)"));
    }

    /**
     * The controller, when nothing registered one.
     *
     * <p>On a Systemcore the system server is a real, separate thing that can be reached or not. On
     * anything else the only evidence the controller exists is that this code is running on it, so
     * "connected" is trivially true and the kind says as much rather than naming hardware it has
     * not identified.
     */
    private static Device defaultController() {
        boolean systemcore;
        try {
            systemcore = SystemCoreStatus.getInstance().isAvailable();
        } catch (RuntimeException e) {
            systemcore = false;
        }
        if (systemcore) {
            return new Device(Kind.CONTROLLER, "Systemcore", "Systemcore", () -> {
                SystemCoreStatus s = SystemCoreStatus.getInstance();
                return s.isAvailable() && s.server() != null && s.server().isConnected();
            });
        }
        return new Device(Kind.CONTROLLER, "Robot controller", "Robot controller", () -> true);
    }

    /** Forget every registration. For tests, which construct hardware more than once per JVM. */
    public static synchronized void clear() {
        devices.clear();
        controller = null;
        lastPublishTs = Double.NaN;
    }

    /** Point publishing at a table other than the default instance's. For tests. */
    static synchronized void useTable(NetworkTable t) {
        table = t;
    }
}
