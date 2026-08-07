package frc.lib.catalyst.identity;

import com.ctre.phoenix6.hardware.traits.CommonDevice;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.WPILibVersion;

import frc.lib.catalyst.hardware.CANRegistry;
import frc.lib.catalyst.hardware.CatalystGyro;
import frc.lib.catalyst.subsystems.swerve.SwerveSubsystem;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.TreeMap;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;

/**
 * The robot's spec sheet, published once at boot for anyone reading NetworkTables.
 *
 * <p>A robot declares who it is; Catalyst works out the rest:
 *
 * <pre>{@code
 * RobotIdentity.declare("Ratchet");
 * }</pre>
 *
 * <p>That one line publishes the team number, the season, which roboRIO this is and its serial, the
 * Catalyst and WPILib versions, the rio image, the brownout threshold, the CAN inventory, and — as
 * soon as a {@link SwerveSubsystem} exists — the drivetrain type, module count and positions, track
 * width, wheelbase, odometry rate, top speed, and the mass and moment of inertia the robot follows
 * paths with. None of that is passed in, because anything passed in can drift: a team that regears
 * its drive and forgets to edit the identity block would publish last month's robot with this
 * month's confidence.
 *
 * <p>The builder exists for the handful of facts no API can answer — a tape measure is the only
 * source for a frame perimeter, and only the team's own build knows which commit their robot code
 * came from:
 *
 * <pre>{@code
 * RobotIdentity.named("Ratchet")
 *     .robotCodeVersion(BuildConstants.GIT_SHA)   // from your own build stamp
 *     .frameMeters(0.74, 0.74)
 *     .bumperThicknessMeters(0.08)
 *     .heightMeters(0.52)
 *     .swerveModules(TunerConstants.FrontLeft, TunerConstants.FrontRight,
 *                    TunerConstants.BackLeft, TunerConstants.BackRight)
 *     .power(pdh)
 *     .publish();
 * }</pre>
 *
 * <h2>What is published, and what is not</h2>
 * Everything lands under {@code /Catalyst/Robot/}, grouped the way a spec sheet is read:
 * {@code Identity}, {@code Software}, {@code Drivetrain}, {@code Chassis}, {@code Power},
 * {@code Hardware}.
 *
 * <p><b>A fact Catalyst does not know is absent from the wire.</b> Not zero, not {@code -1}, not an
 * empty string. A dashboard can tell an absent key from a published one; it cannot tell a placeholder
 * from a measurement, because once a zero is on the wire it looks exactly like a zero that was
 * measured. Catalyst Console leaves an unpublished fact off the sheet rather than dashing it, on the
 * grounds that a dash beside "Gyro" claims the robot has none. A robot with no
 * swerve publishes no {@code Drivetrain} group at all rather than a group full of zeros; a project
 * with no PathPlanner settings publishes no mass. See {@link SpecSheet}, which is built so that
 * omitting a value is easier than inventing one.
 *
 * <h2>Publishing once is enough</h2>
 * NT4 servers keep the last published value of every topic and send it to each subscriber the
 * moment it subscribes. A dashboard that connects in the middle of a match, or reconnects after the
 * radio drops, therefore receives the sheet on connect without the robot republishing anything —
 * which is why this is a boot-time write and not a periodic one, and why nothing here is marked
 * persistent. A persistent topic would be saved to the rio and republished by the *next* boot,
 * so a build that crashed before declaring its identity would serve last week's numbers with a
 * fresh timestamp. Absent is correct; stale-but-confident is the defect.
 *
 * <p>The sheet is re-derived and rewritten whenever a new source of facts turns up — a drivetrain
 * constructed after {@code declare()}, a mechanism claiming CAN ids — so declaration order does not
 * matter.
 *
 * @since 1.10.0
 */
public final class RobotIdentity {

    /** Sub-table under the {@code Catalyst} root. */
    private static final String TABLE = "Robot/";

    /** Where the roboRIO records what the Imaging Tool put on it. Absent on any other machine. */
    private static final Path RIO_IMAGE_FILE = Path.of("/etc/natinst/share/scs_imagemetadata.ini");

    private static final Object LOCK = new Object();

    private static RobotIdentity declared;
    private static SwerveSubsystem drivetrain;
    private static List<String> cameraNames = List.of();
    private static SpecSheet lastSheet;

    // PathPlanner's settings live in a deployed file that has to be read and parsed. It cannot
    // change while the robot is running, so read it once and remember the answer - including the
    // answer "there isn't one", which is why this is a null-until-read Optional rather than an empty
    // one.
    private static Optional<RobotConfig> pathPlannerConfig;

    private final String name;
    private final Optional<String> robotCodeVersion;
    private final Optional<String> robotCodeBuild;
    private final Optional<String> battery;
    private final OptionalDouble frameLengthMeters;
    private final OptionalDouble frameWidthMeters;
    private final OptionalDouble bumperThicknessMeters;
    private final OptionalDouble heightMeters;
    private final SwerveModuleConstants<?, ?, ?>[] swerveModules;
    private final PowerDistribution power;
    private final Map<Integer, String> pdhChannels;

    private RobotIdentity(Builder builder) {
        this.name = builder.name;
        this.robotCodeVersion = builder.robotCodeVersion;
        this.robotCodeBuild = builder.robotCodeBuild;
        this.battery = builder.battery;
        this.frameLengthMeters = builder.frameLengthMeters;
        this.frameWidthMeters = builder.frameWidthMeters;
        this.bumperThicknessMeters = builder.bumperThicknessMeters;
        this.heightMeters = builder.heightMeters;
        this.swerveModules = builder.swerveModules;
        this.power = builder.power;
        this.pdhChannels = new LinkedHashMap<>(builder.pdhChannels);
    }

    // =========================================================
    //                     PUBLIC API
    // =========================================================

    /**
     * Declare the robot by name and publish its spec sheet. This is the whole API for most teams —
     * one line in {@code RobotContainer}, and everything Catalyst can work out for itself appears
     * on the wire.
     *
     * @param robotName what the team calls this robot, e.g. {@code "Ratchet"}
     */
    public static void declare(String robotName) {
        named(robotName).publish();
    }

    /**
     * Start a declaration when there is more to say than the name. Nothing is published until
     * {@link Builder#publish()}.
     *
     * @param robotName what the team calls this robot
     */
    public static Builder named(String robotName) {
        return new Builder(robotName);
    }

    /**
     * The sheet as last published, for tests and for code that wants to log what went out. Empty
     * until a robot has been declared.
     */
    public static Optional<SpecSheet> sheet() {
        synchronized (LOCK) {
            return Optional.ofNullable(lastSheet);
        }
    }

    /**
     * Re-derive the sheet and write it again. Cheap, and a no-op before a robot has been declared.
     *
     * <p>Called for you whenever a new source of facts appears, so that {@code declare()} can sit at
     * the top of {@code RobotContainer} and still pick up a drivetrain built three lines later.
     * Teams should not need to call it.
     */
    public static void refresh() {
        RobotIdentity id;
        synchronized (LOCK) {
            id = declared;
        }
        if (id == null) return;

        SpecSheet sheet = derive(id);
        synchronized (LOCK) {
            lastSheet = sheet;
        }
        sheet.publish(TABLE);
    }

    /**
     * Record the drivetrain the spec sheet should describe.
     *
     * <p>Called for you by {@link SwerveSubsystem}'s constructor; teams do not call this. The first
     * drivetrain wins — a robot has one, and a second registration is more likely a test fixture
     * than a second chassis.
     *
     * @param subsystem the drivetrain that just came up
     */
    public static void observeDrivetrain(SwerveSubsystem subsystem) {
        synchronized (LOCK) {
            if (drivetrain != null || subsystem == null) return;
            drivetrain = subsystem;
        }
        refresh();
    }

    /**
     * Record the cameras the spec sheet should list.
     *
     * <p>Called for you by {@code VisionSubsystem}'s constructor; teams do not call this. Takes
     * names rather than the subsystem so that identity never has to load a vision class — a robot
     * project that excludes PhotonVision still publishes its spec sheet.
     *
     * @param names camera names, in the order the vision subsystem holds them
     */
    public static void observeCameras(List<String> names) {
        if (names == null || names.isEmpty()) return;
        synchronized (LOCK) {
            cameraNames = List.copyOf(names);
        }
        refresh();
    }

    /** Forget everything declared and observed. For tests; production code has one robot. */
    public static void reset() {
        synchronized (LOCK) {
            declared = null;
            drivetrain = null;
            cameraNames = List.of();
            lastSheet = null;
            pathPlannerConfig = null;
        }
        /* Features are observed too, and a test that reset identity but kept them would see the
         * previous test's autopilot on this test's robot. */
        CatalystFeatures.reset();
    }

    /** Builder for the facts Catalyst cannot work out on its own. */
    public static final class Builder {

        private final String name;
        private Optional<String> robotCodeVersion = Optional.empty();
        private Optional<String> robotCodeBuild = Optional.empty();
        private Optional<String> battery = Optional.empty();
        private OptionalDouble frameLengthMeters = OptionalDouble.empty();
        private OptionalDouble frameWidthMeters = OptionalDouble.empty();
        private OptionalDouble bumperThicknessMeters = OptionalDouble.empty();
        private OptionalDouble heightMeters = OptionalDouble.empty();
        private SwerveModuleConstants<?, ?, ?>[] swerveModules = new SwerveModuleConstants<?, ?, ?>[0];
        private PowerDistribution power;
        private final Map<Integer, String> pdhChannels = new LinkedHashMap<>();

        private Builder(String name) {
            if (name == null || name.isBlank()) {
                throw new IllegalArgumentException("robot name is required - it is the one fact only you know");
            }
            this.name = name.trim();
        }

        /**
         * The version of the <em>team's</em> robot code, not Catalyst's.
         *
         * <p>Catalyst can name its own build because it stamps itself at compile time; it cannot
         * name yours, and it will not guess. Wire this to a constant your own build generates (the
         * {@code gversion} Gradle plugin is the usual route) rather than typing a string here — a
         * hand-typed version is wrong the first time anyone forgets to edit it, and a wrong version
         * on a dashboard is worse than none.
         */
        public Builder robotCodeVersion(String version) {
            this.robotCodeVersion = SpecSheet.text(version);
            return this;
        }

        /** The build stamp for the team's robot code — a git sha, a build date, whatever it prints. */
        public Builder robotCodeBuild(String stamp) {
            this.robotCodeBuild = SpecSheet.text(stamp);
            return this;
        }

        /**
         * The swerve module constants Tuner X generated, usually {@code TunerConstants.FrontLeft}
         * and friends.
         *
         * <p>Worth passing because it is the only route to the gear ratios: Phoenix builds a
         * {@code SwerveDrivetrain} from these and then does not hand them back, so a constructed
         * drivetrain can be asked its module positions but not its reduction. Passing the same
         * objects the drivetrain was built from cannot drift out of step with it, which typing the
         * ratios in would.
         *
         * <p>Facts the modules disagree on are dropped rather than averaged — a robot with two
         * different drive reductions has no single drive reduction to publish.
         */
        public Builder swerveModules(SwerveModuleConstants<?, ?, ?>... constants) {
            this.swerveModules = constants == null ? new SwerveModuleConstants<?, ?, ?>[0] : constants.clone();
            return this;
        }

        /** Frame perimeter, in metres, excluding bumpers. Front-to-back then side-to-side. */
        public Builder frameMeters(double lengthMeters, double widthMeters) {
            this.frameLengthMeters = SpecSheet.positive(lengthMeters);
            this.frameWidthMeters = SpecSheet.positive(widthMeters);
            return this;
        }

        /**
         * How far the bumpers stand off the frame, in metres. Combined with
         * {@link #frameMeters(double, double)} this gives the bumper-to-bumper footprint that
         * decides whether the robot fits through a gap.
         */
        public Builder bumperThicknessMeters(double thicknessMeters) {
            this.bumperThicknessMeters = SpecSheet.positive(thicknessMeters);
            return this;
        }

        /** Overall height in metres, at rest — what has to clear a trench rail. */
        public Builder heightMeters(double heightMeters) {
            this.heightMeters = SpecSheet.positive(heightMeters);
            return this;
        }

        /** What battery is in the robot, e.g. {@code "MK ES17-12"}. Nothing can read this. */
        public Builder battery(String description) {
            this.battery = SpecSheet.text(description);
            return this;
        }

        /**
         * The power distribution module, for its type and channel count.
         *
         * <p>Pass the same object the rest of the robot uses. Constructing a second
         * {@link PowerDistribution} for the sake of the spec sheet claims the HAL resource twice.
         */
        public Builder power(PowerDistribution powerDistribution) {
            this.power = powerDistribution;
            return this;
        }

        /**
         * Label one power distribution channel. Repeat per channel in use.
         *
         * <p>The module knows how many channels it has and how much current each is drawing; it does
         * not know what is wired to them, and current draw at boot is zero on a channel that is in
         * use and zero on one that is not. So the wiring map is declared or it is absent.
         *
         * @param channel channel number on the module
         * @param what    what is wired to it, e.g. {@code "Front left drive"}
         */
        public Builder pdhChannel(int channel, String what) {
            SpecSheet.text(what).ifPresent(label -> pdhChannels.put(channel, label));
            return this;
        }

        /** Publish the sheet. Safe to call again; it overwrites rather than accumulating. */
        public void publish() {
            RobotIdentity identity = build();
            synchronized (LOCK) {
                declared = identity;
            }
            refresh();
        }

        /** The declaration, without publishing it. Lets a test derive a sheet with no NT anywhere. */
        RobotIdentity build() {
            return new RobotIdentity(this);
        }
    }

    // =========================================================
    //                     DERIVATION
    // =========================================================

    /**
     * Build the sheet without writing it anywhere.
     *
     * <p>Package-visible so a test can assemble a whole sheet and check what is missing from it.
     * Every source that needs a HAL is wrapped, so this runs on a bare JVM and simply reports the
     * facts that machine does not have — which is the behaviour worth testing.
     */
    static SpecSheet derive(RobotIdentity id) {
        SwerveSubsystem swerve;
        List<String> cameras;
        synchronized (LOCK) {
            swerve = drivetrain;
            cameras = cameraNames;
        }

        SpecSheet sheet = new SpecSheet();
        addIdentity(sheet, id);
        addSoftware(sheet, id);
        addDrivetrain(sheet, id, swerve);
        addChassis(sheet, id);
        addPower(sheet, id);
        addHardware(sheet, swerve, cameras);
        addCatalyst(sheet);
        return sheet;
    }

    private static void addIdentity(SpecSheet sheet, RobotIdentity id) {
        sheet.putText("Identity/Name", Optional.of(id.name));
        // getTeamNumber() reports 0 on a rio that was never given one, and on every desktop.
        sheet.putInt("Identity/TeamNumber", positiveInt(RobotController::getTeamNumber));
        sheet.putInt("Identity/Season", season());
        sheet.putText("Identity/Controller", controller());
        sheet.putText("Identity/RioSerial", text(RobotController::getSerialNumber));
        // The rio's "comments" field from the Imaging Tool - the only free text the controller
        // holds, and often where a team already wrote the robot's name.
        sheet.putText("Identity/RioComment", text(RobotController::getComments));
    }

    private static void addSoftware(SpecSheet sheet, RobotIdentity id) {
        sheet.putText("Software/CatalystVersion", Optional.of(CatalystVersion.version()));
        sheet.putText("Software/CatalystGitSha", CatalystVersion.gitSha());
        CatalystVersion.isDirty().ifPresent(dirty -> sheet.putFlag("Software/CatalystGitDirty", dirty));
        sheet.putText("Software/CatalystCommitTime", CatalystVersion.commitTime());

        sheet.putText("Software/RobotCodeVersion", id.robotCodeVersion);
        sheet.putText("Software/RobotCodeBuild", id.robotCodeBuild);

        // The runtime constant, not the version build.gradle pinned: a robot project resolves its own
        // WPILib, and Catalyst's build-time pin says nothing about which one is loaded here.
        sheet.putText("Software/WPILibVersion", SpecSheet.text(WPILibVersion.Version));
        sheet.putText("Software/JavaVersion", SpecSheet.text(System.getProperty("java.version")));
        sheet.putText("Software/RioImage", rioImage());
        sheet.putInt("Software/FpgaVersion", positiveInt(RobotController::getFPGAVersion));

        // Phoenix, PathPlanner and PhotonVision are deliberately not here. Their versions are pinned
        // in Catalyst's build.gradle and nowhere else - none of the three publishes a runtime
        // constant - and a robot resolves its own copies. Reporting the pins would be reporting
        // which versions Catalyst was compiled against as though they were the ones running.
    }

    private static void addDrivetrain(SpecSheet sheet, RobotIdentity id, SwerveSubsystem swerve) {
        // No drivetrain, no group. The package tree holds no tank, differential, mecanum or arcade
        // class, so a SwerveSubsystem is the only drivetrain Catalyst can recognise - and a robot
        // driving on something else deserves silence rather than a swerve-shaped guess.
        if (swerve == null) return;

        sheet.putText("Drivetrain/Type", Optional.of("Swerve"));
        sheet.putNumber("Drivetrain/MaxSpeedMps", SpecSheet.positive(swerve.getMaxSpeedMPS()));
        // The rate the subsystem actually limits rotation to. Unless setMaxAngularRate() overrode it
        // this is maxSpeed divided by the first module's radius, which is exact for a symmetric
        // layout and is in any case the number the robot drives by.
        sheet.putNumber("Drivetrain/MaxAngularRateRadPerSec",
                SpecSheet.positive(swerve.getMaxAngularRate()));

        SwerveDrivetrain raw = swerve.getDrivetrain();
        if (raw != null) {
            Translation2d[] locations = raw.getModuleLocations();
            if (locations != null && locations.length > 0) {
                sheet.putInt("Drivetrain/Modules", SpecSheet.positive(locations.length));
                sheet.putNumbers("Drivetrain/ModuleLocations", flatten(locations));
                rectangularSpan(locations).ifPresent(span -> {
                    sheet.putNumber("Drivetrain/TrackWidthMeters", OptionalDouble.of(span[0]));
                    sheet.putNumber("Drivetrain/WheelBaseMeters", OptionalDouble.of(span[1]));
                });
            }
            sheet.putNumber("Drivetrain/OdometryHz", SpecSheet.positive(raw.getOdometryFrequency()));
            sheet.putFlag("Drivetrain/CanFd", raw.isOnCANFD());
        }

        // Gear ratios and wheel radius only exist if the team handed over the module constants.
        SwerveModuleConstants<?, ?, ?>[] modules = id.swerveModules;
        if (modules.length > 0) {
            sheet.putNumber("Drivetrain/DriveGearRatio", agreed(modules, c -> c.DriveMotorGearRatio));
            sheet.putNumber("Drivetrain/SteerGearRatio", agreed(modules, c -> c.SteerMotorGearRatio));
            sheet.putNumber("Drivetrain/WheelRadiusMeters", agreed(modules, c -> c.WheelRadius));
            sheet.putNumber("Drivetrain/SlipCurrentAmps", agreed(modules, c -> c.SlipCurrent));
        } else {
            // PathPlanner's GUI settings carry a wheel radius too, and it is the one the robot
            // follows paths with, so it is worth having when the module constants are not offered.
            pathPlannerConfig().ifPresent(config -> sheet.putNumber("Drivetrain/WheelRadiusMeters",
                    SpecSheet.positive(config.moduleConfig.wheelRadiusMeters)));
        }

        pathPlannerConfig().ifPresent(config -> {
            sheet.putNumber("Drivetrain/WheelCof", SpecSheet.positive(config.moduleConfig.wheelCOF));
            sheet.putNumber("Drivetrain/DriveCurrentLimitAmps",
                    SpecSheet.positive(config.moduleConfig.driveCurrentLimit));
        });
    }

    private static void addChassis(SpecSheet sheet, RobotIdentity id) {
        // Mass and inertia come from the deployed PathPlanner settings, which is the one place a
        // team has already typed them and the numbers the robot's own path follower runs on. When
        // the project has no settings file the keys are simply absent - Catalyst does not hold a
        // fallback mass, and RobotModel is not consulted, because RobotModel fills its unset fields
        // with defaults and cannot say afterwards which figures the team actually measured.
        pathPlannerConfig().ifPresent(config -> {
            sheet.putNumber("Chassis/MassKg", SpecSheet.positive(config.massKG));
            sheet.putNumber("Chassis/MoiKgM2", SpecSheet.positive(config.MOI));
        });

        sheet.putNumber("Chassis/FrameLengthMeters", id.frameLengthMeters);
        sheet.putNumber("Chassis/FrameWidthMeters", id.frameWidthMeters);
        sheet.putNumber("Chassis/BumperThicknessMeters", id.bumperThicknessMeters);
        sheet.putNumber("Chassis/HeightMeters", id.heightMeters);

        // Bumper-to-bumper is arithmetic on two declared numbers, so it appears only when both do.
        if (id.bumperThicknessMeters.isPresent()) {
            double growth = 2 * id.bumperThicknessMeters.getAsDouble();
            id.frameLengthMeters.ifPresent(length ->
                    sheet.putNumber("Chassis/BumperLengthMeters", OptionalDouble.of(length + growth)));
            id.frameWidthMeters.ifPresent(width ->
                    sheet.putNumber("Chassis/BumperWidthMeters", OptionalDouble.of(width + growth)));
        }
    }

    private static void addPower(SpecSheet sheet, RobotIdentity id) {
        sheet.putNumber("Power/BrownoutVolts", number(RobotController::getBrownoutVoltage));
        sheet.putText("Power/Battery", id.battery);

        if (id.power != null) {
            sheet.putText("Power/Module", moduleName(id.power));
            sheet.putInt("Power/Channels", positiveInt(id.power::getNumChannels));
        }

        if (!id.pdhChannels.isEmpty()) {
            // "channel|what", matching how CANRegistry serialises a device list. Composite rows on
            // this wire are pipe-delimited strings, not JSON.
            List<String> rows = new ArrayList<>();
            new TreeMap<>(id.pdhChannels).forEach((channel, what) -> rows.add(channel + "|" + what));
            sheet.putList("Power/ChannelsInUse", rows);
        }
    }

    private static void addHardware(SpecSheet sheet, SwerveSubsystem swerve, List<String> cameras) {
        Map<String, String> devices = canDevices(swerve);
        if (!devices.isEmpty()) {
            sheet.putInt("Hardware/CanDevices", SpecSheet.positive(devices.size()));
            // "type|count", one row per kind of device on the robot.
            Map<String, Integer> counts = new TreeMap<>();
            devices.values().forEach(type -> counts.merge(type, 1, Integer::sum));
            List<String> rows = new ArrayList<>();
            counts.forEach((type, count) -> rows.add(type + "|" + count));
            sheet.putList("Hardware/Inventory", rows);
        }

        gyro(swerve).ifPresent(found -> {
            sheet.putText("Hardware/Gyro", Optional.of(found.name()));
            // CAN id 0 is a legal id, so this one is guarded against negatives rather than against
            // zero - SpecSheet.positive() would quietly drop a device addressed at 0.
            sheet.putInt("Hardware/GyroCanId",
                    found.canId() >= 0 ? OptionalInt.of(found.canId()) : OptionalInt.empty());
        });

        sheet.putList("Hardware/Cameras", cameras);
    }

    /**
     * What this robot is made to do, as opposed to what it is made of.
     *
     * <p>Every entry was written by the component that created it — see {@link CatalystFeatures} —
     * so this is a list of what came up rather than a list of what a team meant to use. A part of
     * the library the robot does not run is absent, not published as a false flag: "no autopilot"
     * and "an autopilot that failed to build" would otherwise look identical on the wire, and only
     * one of those is worth a driver's attention.
     */
    private static void addCatalyst(SpecSheet sheet) {
        Map<String, List<String>> features = CatalystFeatures.snapshot();
        if (features.isEmpty()) return;

        sheet.putList("Catalyst/InUse", List.copyOf(features.keySet()));

        features.forEach((feature, labels) -> {
            // "Physics Core" -> "PhysicsCore", so the key is a key and the label stays readable.
            // The constants are capitalised per word for this reason; see CatalystFeatures.
            String key = feature.replace(" ", "");
            sheet.putInt("Catalyst/" + key + "/Count", OptionalInt.of(Math.max(1, labels.size())));
            sheet.putList("Catalyst/" + key + "/Names", labels);
        });
    }

    /**
     * Every CAN device this robot program is driving, keyed by {@code bus/id}.
     *
     * <p>Two sources, because neither is complete. {@link CANRegistry} holds what the mechanisms
     * claimed as they were built, and misses the drivetrain entirely: Phoenix constructs the module
     * motors and encoders inside {@code SwerveDrivetrain} and nothing tells the registry about them,
     * so a registry-only count reports a swerve robot's twelve largest motors as none. The
     * drivetrain, in turn, knows nothing about the mechanisms. Merging on {@code bus/id} means a
     * device counted twice is still counted once.
     *
     * <p>Types are the devices' own class names rather than a guess — a module built on a TalonFXS
     * reports a TalonFXS.
     */
    private static Map<String, String> canDevices(SwerveSubsystem swerve) {
        Map<String, String> byKey = new LinkedHashMap<>();
        for (CANRegistry.Entry device : CANRegistry.all()) {
            String type = device.type() == null || device.type().isBlank() ? "Unknown" : device.type();
            byKey.put(busKey(device.bus(), device.canId()), type);
        }

        SwerveDrivetrain raw = swerve == null ? null : swerve.getDrivetrain();
        if (raw != null) {
            try {
                for (Object module : raw.getModules()) {
                    com.ctre.phoenix6.swerve.SwerveModule<?, ?, ?> m =
                            (com.ctre.phoenix6.swerve.SwerveModule<?, ?, ?>) module;
                    addDevice(byKey, m.getDriveMotor());
                    addDevice(byKey, m.getSteerMotor());
                    addDevice(byKey, m.getEncoder());
                }
                addDevice(byKey, raw.getPigeon2());
            } catch (Throwable ignored) {
                // Phoenix is not answering. The mechanisms' own claims still stand.
            }
        }
        return byKey;
    }

    private static void addDevice(Map<String, String> byKey, CommonDevice device) {
        if (device == null) return;
        String bus = device.getNetwork() == null ? "" : device.getNetwork().getName();
        byKey.putIfAbsent(busKey(bus, device.getDeviceID()), device.getClass().getSimpleName());
    }

    /** Phoenix names the rio's own bus "rio"; CANRegistry spells the same bus "". */
    private static String busKey(String bus, int canId) {
        String normalised = bus == null || bus.isBlank() || "rio".equals(bus) ? "" : bus;
        return normalised + "/" + canId;
    }

    // =========================================================
    //                 SOURCES THAT CAN FAIL
    // =========================================================

    /**
     * Read a string from a source that needs the HAL.
     *
     * <p>Every one of these throws {@link UnsatisfiedLinkError} rather than an exception on a
     * machine with no HAL loaded — a unit test, a desktop tool — so the catch has to reach
     * {@link Throwable}. The right answer there is "unknown", which is exactly what an empty
     * {@code Optional} publishes as.
     */
    private static Optional<String> text(Supplier<String> source) {
        try {
            return SpecSheet.text(source.get());
        } catch (Throwable ignored) {
            return Optional.empty();
        }
    }

    private static OptionalDouble number(java.util.function.DoubleSupplier source) {
        try {
            return SpecSheet.positive(source.getAsDouble());
        } catch (Throwable ignored) {
            return OptionalDouble.empty();
        }
    }

    private static OptionalInt positiveInt(java.util.function.IntSupplier source) {
        try {
            return SpecSheet.positive(source.getAsInt());
        } catch (Throwable ignored) {
            return OptionalInt.empty();
        }
    }

    /** Which controller this is, from the HAL runtime type. */
    private static Optional<String> controller() {
        try {
            return switch (HALUtil.getHALRuntimeType()) {
                case HALUtil.RUNTIME_ROBORIO -> Optional.of("roboRIO");
                case HALUtil.RUNTIME_ROBORIO2 -> Optional.of("roboRIO 2");
                case HALUtil.RUNTIME_SIMULATION -> Optional.of("Simulation");
                default -> Optional.empty();
            };
        } catch (Throwable ignored) {
            return Optional.empty();
        }
    }

    /**
     * The competition season, taken from the WPILib version's leading year.
     *
     * <p>WPILib ships one release line per season and the robot cannot run against another, so the
     * year in {@code 2026.2.1} is the season this code is for. Absent if the string ever stops
     * starting with a year.
     */
    private static OptionalInt season() {
        String version = WPILibVersion.Version;
        if (version == null || version.length() < 4) return OptionalInt.empty();
        try {
            return SpecSheet.positive(Integer.parseInt(version.substring(0, 4)));
        } catch (NumberFormatException ignored) {
            return OptionalInt.empty();
        }
    }

    /**
     * The roboRIO image, read off the controller itself.
     *
     * <p>WPILib exposes no API for this, but the Imaging Tool leaves it in a known ini file on the
     * rio. Anywhere else — a desktop, a simulation — the file does not exist and the key does not
     * appear, which is the correct answer rather than a shortcoming.
     */
    private static Optional<String> rioImage() {
        try {
            if (!Files.isReadable(RIO_IMAGE_FILE)) return Optional.empty();
            for (String line : Files.readAllLines(RIO_IMAGE_FILE, StandardCharsets.UTF_8)) {
                String trimmed = line.trim();
                if (!trimmed.toUpperCase().startsWith("IMAGEVERSION")) continue;
                int equals = trimmed.indexOf('=');
                if (equals < 0) continue;
                return SpecSheet.text(trimmed.substring(equals + 1).trim().replace("\"", ""));
            }
            return Optional.empty();
        } catch (Exception ignored) {
            return Optional.empty();
        }
    }

    /** PathPlanner's deployed settings, read at most once. */
    private static Optional<RobotConfig> pathPlannerConfig() {
        synchronized (LOCK) {
            if (pathPlannerConfig != null) return pathPlannerConfig;
        }
        Optional<RobotConfig> config;
        try {
            RobotConfig loaded = RobotConfig.fromGUISettings();
            config = loaded != null && loaded.hasValidConfig() ? Optional.of(loaded) : Optional.empty();
        } catch (Throwable ignored) {
            // No deployed settings.json, or no deploy directory at all. The team has not told
            // PathPlanner what the robot weighs, so nobody knows what the robot weighs.
            config = Optional.empty();
        }
        synchronized (LOCK) {
            pathPlannerConfig = config;
        }
        return config;
    }

    /** A gyro and its CAN id: the drivetrain's Pigeon 2 if there is one, else the CAN registry's. */
    private static Optional<Gyro> gyro(SwerveSubsystem swerve) {
        if (swerve != null && swerve.getDrivetrain() != null) {
            try {
                var pigeon = swerve.getDrivetrain().getPigeon2();
                if (pigeon != null) {
                    return Optional.of(new Gyro(CatalystGyro.DEVICE_TYPE, pigeon.getDeviceID()));
                }
            } catch (Throwable ignored) {
                // Fall through to the registry.
            }
        }
        for (CANRegistry.Entry device : CANRegistry.all()) {
            if (CatalystGyro.DEVICE_TYPE.equals(device.type())) {
                return Optional.of(new Gyro(device.type(), device.canId()));
            }
        }
        return Optional.empty();
    }

    private record Gyro(String name, int canId) {}

    private static Optional<String> moduleName(PowerDistribution power) {
        try {
            return switch (power.getType()) {
                case kCTRE -> Optional.of("CTRE PDP");
                case kRev -> Optional.of("REV PDH");
                default -> Optional.empty();
            };
        } catch (Throwable ignored) {
            return Optional.empty();
        }
    }

    // =========================================================
    //                      GEOMETRY
    // =========================================================

    /** Module positions as {@code [x0, y0, x1, y1, ...]}, robot-relative, in metres. */
    private static double[] flatten(Translation2d[] locations) {
        double[] flat = new double[locations.length * 2];
        for (int i = 0; i < locations.length; i++) {
            flat[2 * i] = locations[i].getX();
            flat[2 * i + 1] = locations[i].getY();
        }
        return flat;
    }

    /**
     * Track width and wheelbase, but only from a layout that actually has them.
     *
     * <p>{@code 2*max|y|} and {@code 2*max|x|} describe a rectangle. Handed a diamond, a six-module
     * chassis or anything else asymmetric they still produce two plausible numbers, describing a
     * robot that does not exist. So the layout is checked first: every module the same distance out
     * on each axis, four or more of them, and an even count. Anything else publishes its module
     * positions — which are exact — and no span.
     *
     * @return {@code [trackWidth, wheelBase]} in metres, or empty when the layout is not rectangular
     */
    static Optional<double[]> rectangularSpan(Translation2d[] locations) {
        if (locations == null || locations.length < 4 || locations.length % 2 != 0) {
            return Optional.empty();
        }
        double halfWheelBase = Math.abs(locations[0].getX());
        double halfTrackWidth = Math.abs(locations[0].getY());
        if (halfWheelBase <= 0 || halfTrackWidth <= 0) return Optional.empty();

        // A millimetre: tighter than any real chassis is built, looser than floating point noise.
        final double tolerance = 1e-3;
        for (Translation2d location : locations) {
            if (Math.abs(Math.abs(location.getX()) - halfWheelBase) > tolerance) return Optional.empty();
            if (Math.abs(Math.abs(location.getY()) - halfTrackWidth) > tolerance) return Optional.empty();
        }
        return Optional.of(new double[] {2 * halfTrackWidth, 2 * halfWheelBase});
    }

    /**
     * A value every module agrees on, or nothing.
     *
     * <p>Modules that disagree have no shared answer, and an average of two gear ratios is a ratio
     * no module on the robot is built to.
     */
    private static OptionalDouble agreed(SwerveModuleConstants<?, ?, ?>[] modules,
                                         ToDoubleFunction<SwerveModuleConstants<?, ?, ?>> reader) {
        if (modules.length == 0) return OptionalDouble.empty();
        double first = reader.applyAsDouble(modules[0]);
        if (!Double.isFinite(first) || first <= 0) return OptionalDouble.empty();
        for (SwerveModuleConstants<?, ?, ?> module : modules) {
            if (Math.abs(reader.applyAsDouble(module) - first) > 1e-9) return OptionalDouble.empty();
        }
        return OptionalDouble.of(first);
    }

    @Override
    public String toString() {
        return "RobotIdentity[" + name + ", " + (lastSheet == null ? 0 : lastSheet.size()) + " facts]";
    }
}
