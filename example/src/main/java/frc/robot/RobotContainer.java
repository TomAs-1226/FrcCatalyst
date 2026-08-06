package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.catalyst.behavior.Action;
import frc.lib.catalyst.behavior.Strategist;
import frc.lib.catalyst.goal.Goal;
import frc.lib.catalyst.goal.GoalDirector;
import edu.wpi.first.math.geometry.Translation3d;

import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.logging.CatalystLog;
import frc.lib.catalyst.physics.contact.CollisionField;
import frc.lib.catalyst.physics.contact.ContactMaterial;
import frc.lib.catalyst.physics.contact.ContactResolver;
import frc.lib.catalyst.physics.contact.FieldHeightmap;
import frc.lib.catalyst.mechanisms.FlywheelMechanism;
import frc.lib.catalyst.mechanisms.LinearMechanism;
import frc.lib.catalyst.mechanisms.RollerMechanism;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.lib.catalyst.mechanisms.TurretMechanism;
import frc.lib.catalyst.util.AimingSolver;
import frc.lib.catalyst.util.AlertManager;
import frc.lib.catalyst.util.GhostReplay;
import frc.lib.catalyst.util.InterpolatingTable;
import frc.lib.catalyst.util.LoopMonitor;
import frc.lib.catalyst.util.RobotSafety;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Catalyst demo robot — a 2026 fuel shooter driven by the optional
 * {@link GoalDirector} goal layer.
 *
 * <p>The driver gives <em>intent</em>, not motor commands:
 * <ul>
 *   <li><b>Intake</b> — run the intake to collect fuel</li>
 *   <li><b>Aim &amp; Shoot</b> — the turret tracks the goal with Shoot-On-The-Fly
 *       lead, the flywheel spins to the distance-correct RPM, the hood sets its
 *       angle, and it fires when everything is ready</li>
 *   <li><b>Idle</b> — stand down</li>
 * </ul>
 *
 * <p>Drive the chassis around with <b>WASD / arrow keys</b> in the cockpit and
 * watch the turret keep its barrel on the (motion-compensated) virtual goal —
 * that's {@link AimingSolver} doing the SOTF math live. Everything self
 * simulates: the turret, flywheel, intake and hood all run WPILib physics, so
 * the robot moves with no hardware.
 *
 * <p>Open the cockpit at <a href="http://localhost:5805">localhost:5805</a>.
 */
public class RobotContainer {

    // ===================== REBUILT (2026) field model =====================
    // Researched from the FIRST REBUILT game manual + field drawings. The field
    // is 54 ft x 27 ft, taken here as 16.54 m x 8.07 m. Each alliance HUB is a
    // 47" structure 158.6" (4.03 m) from its alliance wall; robots shoot Fuel up
    // into the hex opening on top. BUMPS (73", 15deg, traversable) flank each hub;
    // TRENCHES run the perimeter; the DEPOT/OUTPOST feed fuel; the TOWER is the
    // endgame climb. (Geometry is faithful to the manual; exact CAD coords drop
    // in here.)
    //
    // This comment used to quote 8.21 m while the constant below said 8.07 — it
    // read as the justification for a number it contradicted.
    /**
     * Field size to use when <em>no</em> CAD map is deployed, and nothing else.
     *
     * <p>The deployed heightmap header is the authority whenever there is one. Read these two
     * directly on a path that also runs with a map loaded and the two descriptions of the field drift
     * apart; call {@link #fieldLength()} / {@link #fieldWidth()} instead. SimCockpit names its pair
     * {@code FW_FALLBACK} / {@code FH_FALLBACK} for exactly this reason — these carried no such marker,
     * which is how the opponent clamp and the /state feed kept reading them.
     */
    private static final double FIELD_W = 16.54, FIELD_H = 8.07;
    /** Half the bumper-to-bumper size of this (square) 0.9 m robot, in metres. */
    private static final double ROBOT_HALF_EXTENT = 0.45;
    /**
     * The same robot expressed as a disc, for the tests that only know about circles.
     *
     * <p>A square does not fit inside a circle of its half-extent — the corners reach a further 41% out,
     * 0.186 m here. The circular hub, tower and opponent tests below used to be handed the half-extent,
     * so a corner could sink that far into a hub before anything pushed back. Box tests (the heightmap,
     * {@link CollisionField}) want the half-extent; circle tests want this.
     */
    private static final double ROBOT_RADIUS = ROBOT_HALF_EXTENT * Math.sqrt(2.0);
    private static final double ROBOT_MASS_KG = 55.0;

    /**
     * The solid perimeter, with the two alliance hubs in it.
     *
     * <p>Built from the library's own {@link CollisionField} so the showcase drives against the same
     * contact physics a team's simulation would, rather than a private approximation that can quietly
     * drift away from it.
     */
    /**
     * Collision taken from the season CAD, when the generated map has been deployed.
     *
     * <p>This is what makes the ramp climbable and the trench a real passage. The box list below only
     * ever knew about a rectangle and two hubs, so everything else on the field — every ramp, every
     * trench wall — simply was not there to hit.
     */
    private static final FieldHeightmap HEIGHTMAP = FieldHeightmap.tryLoad(
            Filesystem.getDeployDirectory().toPath().resolve("field-collision.json")).orElse(null);

    /**
     * How long the field is, along +x, in metres: the deployed map when there is one, the fallback
     * constant otherwise.
     *
     * <p>Every runtime use of the field size goes through this and {@link #fieldWidth()}. The opponent
     * clamp and the /state feed used to read the constants straight, so a deployment with a map got an
     * opponent penned into the wrong rectangle and a state feed asserting a field size the map
     * contradicted.
     */
    private static double fieldLength() {
        return HEIGHTMAP != null ? HEIGHTMAP.lengthMeters() : FIELD_W;
    }

    /** How wide the field is, along +y, in metres. See {@link #fieldLength()}. */
    private static double fieldWidth() {
        return HEIGHTMAP != null ? HEIGHTMAP.widthMeters() : FIELD_H;
    }

    private static final double ROBOT_TALL = 0.85;

    // The constants are right here: this whole model is the no-map fallback, and is only ever
    // consulted below when HEIGHTMAP is null.
    private static final CollisionField COLLISION = CollisionField.rectangular(FIELD_W, FIELD_H)
            .addObstacle("blue hub", new Translation2d(4.03, FIELD_H / 2), 0.6, 0.6,
                    ContactMaterial.ALUMINIUM)
            .addObstacle("red hub", new Translation2d(FIELD_W - 4.03, FIELD_H / 2), 0.6, 0.6,
                    ContactMaterial.ALUMINIUM)
            .build();
    private static final double ALLIANCE_ZONE = 4.03;    // zone depth from each wall (m)

    private static final Translation2d RED_HUB  = new Translation2d(4.03, 4.10);
    private static final Translation2d BLUE_HUB = new Translation2d(12.51, 4.10);
    private static final double HUB_R = 0.60;            // 47" structure ~ 1.19 m
    // Towers sit at the field edges (against the alliance walls).
    private static final Translation2d RED_TOWER  = new Translation2d(0.85, 4.10);
    private static final Translation2d BLUE_TOWER = new Translation2d(15.69, 4.10);
    private static final double TOWER_R = 0.42;

    // Our robot is on the BLUE alliance and scores into the BLUE hub.
    //
    // Known seam, not an oversight: this is a hand-placed coordinate, and the cockpit now resolves its
    // hub cues against the deployed CAD map. With a map loaded the two can disagree — aiming and the
    // 0.8 m hit test in fireShot() work off the number here while the hub is drawn where the map puts
    // it, so a shot can score while looking like it missed (or the reverse). Reconciling them means
    // sourcing the hub from the map, which is a larger change than anyone has signed up for; until
    // then, believe this coordinate for scoring and the map for drawing.
    private static final Translation2d GOAL = BLUE_HUB;

    /** Impassable structures: {centerX, centerY, radius}. Hubs + towers. */
    private static final double[][] OBSTACLES = {
            {RED_HUB.getX(),   RED_HUB.getY(),   HUB_R},
            {BLUE_HUB.getX(),  BLUE_HUB.getY(),  HUB_R},
            {RED_TOWER.getX(), RED_TOWER.getY(), TOWER_R},
            {BLUE_TOWER.getX(),BLUE_TOWER.getY(),TOWER_R},
    };

    private static final double AUTO_SECONDS = 20.0;     // REBUILT autonomous period
    private static final double MATCH_SECONDS = 160.0;   // 20 s auto + 140 s teleop
    /**
     * Time remaining at the end of each TELEOP segment, from the 2026 Game Manual, Table 6-2:
     * transition shift, then shifts 1-4, then END GAME.
     */
    private static final double[] SHIFT_ENDS = {130.0, 105.0, 80.0, 55.0, 30.0, 0.0};

    /** Disabled pause between looped matches, so the transition is visible rather than instant. */
    private static final double RESET_SECONDS = 8.0;

    /** Measures the real loop time and publishes it under {@code Catalyst/Loop/Robot/...}. */
    private final LoopMonitor loop = new LoopMonitor();
    private static final int MAX_FUEL = 50;

    // --- Mechanisms ---------------------------------------------------------

    private final RollerMechanism intake = new RollerMechanism(
            RollerMechanism.Config.builder()
                    .name("Intake")
                    .motor(20)
                    .intakeSpeed(0.9)
                    .ejectSpeed(-0.6)
                    .currentLimit(30)
                    .stallDetection(25, 0.2)
                    .build());

    private final TurretMechanism turret = new TurretMechanism(
            TurretMechanism.Config.builder()
                    .name("Turret")
                    .motor(21)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(30.0)
                    .range(-270, 270)
                    .currentLimit(40)
                    .pid(60, 0, 0.8)
                    .feedforward(0.12, 3.0)    // realistic kV (V per turret-rps) so velocity FF leads
                    .motionMagic(16, 60, 600)
                    .tolerance(3.0)             // shootable while tracking on the move
                    .simMOI(0.03)
                    .build());

    private final FlywheelMechanism shooter = new FlywheelMechanism(
            FlywheelMechanism.Config.builder()
                    .name("Shooter")
                    .motor(22)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(1.0)
                    .moi(0.0016)                 // light wheel = fast RPM recovery
                    .currentLimit(80)
                    .pid(0.45, 0, 0)
                    .feedforward(0.18, 0.119)
                    .velocityTolerance(2.0)
                    .build());

    private final RotationalMechanism hood = new RotationalMechanism(
            RotationalMechanism.Config.builder()
                    .name("Hood")
                    .motor(23)
                    .motorType(MotorType.KRAKEN_X44)
                    .gearRatio(60.0)
                    .length(0.25)
                    .mass(1.0)
                    .range(0, 50)
                    .currentLimit(25)
                    .pid(60, 0, 1.0)
                    .gravityGain(0.10)
                    .motionMagic(200, 400, 3000)
                    .build());

    // Endgame Tower climber (real LinearMechanism / winch).
    private final LinearMechanism climber = new LinearMechanism(
            LinearMechanism.Config.builder()
                    .name("Climber")
                    .motor(24)
                    .motorType(MotorType.KRAKEN_X60)
                    .gearRatio(12.0)
                    .drumRadius(0.022)
                    .range(0.0, 0.60)
                    .mass(9.0)
                    .currentLimit(60)
                    .pid(60, 0, 0)
                    .motionMagic(1.4, 3.0, 30.0)
                    .position("DOWN", 0.0)
                    .position("UP", 0.55)
                    .build());

    // --- SOTF aiming --------------------------------------------------------

    private final AimingSolver solver = AimingSolver.builder()
            .target(GOAL)
            .shotTime(new InterpolatingTable().add(1.5, 0.25).add(4.0, 0.45).add(8.0, 0.80))
            .shooterRpm(new InterpolatingTable().add(1.5, 2500).add(4.0, 3500).add(8.0, 4800))
            .hoodAngle(new InterpolatingTable().add(1.5, 10).add(4.0, 28).add(8.0, 44))
            .iterations(3)
            .maxRange(4.0)        // must be in the scoring zone near the hub — no neutral-middle shots
            .build();

    private volatile AimingSolver.Solution solution =
            solver.solveStatic(new Pose2d(8.0, 4.0, new Rotation2d()));

    // --- Game state (declared before goals: their lambdas reference it) ------

    private int fuel = 0;
    private int shots = 0;
    private int hits = 0;
    private int misses = 0;
    private int idealHits = 0;     // hits if turret + flywheel tracked perfectly (proves the math)
    private int scored = 0;        // fuel scored (hit while the hub was active)
    private boolean firing = false;
    private double lastActionTs = 0;

    private record ShotRec(int id, double x0, double y0, double lx, double ly, boolean hit, double tof) {}
    private final java.util.ArrayDeque<ShotRec> recentShots = new java.util.ArrayDeque<>();

    // --- Goal layer (optional) ----------------------------------------------

    private final Goal IDLE = Goal.named("Idle").build();

    private final Goal INTAKE = Goal.named("Intake")
            .with(intake::intakeContinuous)
            .readyWhen(() -> fuel > 0)
            .build();

    private final Goal AIM_SHOOT = Goal.named("AimShoot")
            .with(() -> Commands.parallel(
                    // Live tracking: RPM and hood follow the distance EVERY loop,
                    // so radial motion (toward/away from the goal) is compensated
                    // too — no SOTF deadzone.
                    shooter.track(() -> solution.shooterRpm() / 60.0),  // RPM -> RPS
                    hood.track(() -> solution.hoodDegrees())))
            .readyWhen(() -> shooter.atSpeed() && turret.atSetpoint()
                    && solution.feasible() && fuel > 0)
            .build();

    private final GoalDirector director = GoalDirector.builder()
            .defaultGoal(IDLE)
            .build();

    // --- Simulated holonomic chassis (no swerve hardware needed) -------------

    private static final double START_X = 9.5, START_Y = 5.50;
    private double poseX = START_X, poseY = START_Y;   // meters (neutral zone)
    private double heading = 0.0;                       // radians (facing +x, toward the blue hub)
    private volatile double cmdVx = 0, cmdVy = 0;  // field m/s requested from the cockpit
    private volatile double cmdOmega = 0;          // rad/s requested
    private double velX = 0, velY = 0;             // actual field velocity (slew-limited)
    private double omega = 0;                       // actual yaw rate (rad/s)

    // Autonomous routine state.
    private boolean autoRunning = false;
    private double autoT0 = 0;
    private int autoIdx = 0;
    private int autoScored = 0;
    private double autoWpReachedT = -1;
    private boolean hubActive = true;
    /** How far up a ramp the robot is sitting, in metres. */
    private double groundHeight = 0.0;
    private double lastTrailT = 0;
    private final java.util.ArrayDeque<double[]> trail = new java.util.ArrayDeque<>();

    // Autonomous plan: {x, y, mode} — mode 0=transit, 1=intake, 2=shoot.
    // A blue robot collects Fuel from the center field, then scores from the
    // BLUE scoring zone next to its hub (you can't score from the neutral
    // middle). Transit waypoints route AROUND the hub obstacle.
    private static final double[][] AUTO_PLAN = {
            {8.00, 5.40, 1},   // intake — top of the center fuel field
            {11.2, 5.70, 0},   // route around the hub (high lane)
            {13.5, 5.25, 2},   // shoot — blue scoring zone, beside the hub
            {11.2, 5.70, 0},   // route back (high lane)
            {8.20, 2.95, 1},   // intake — bottom of the fuel field
            {11.2, 2.55, 0},   // route around the hub (low lane)
            {13.5, 3.00, 2},   // shoot — blue scoring zone
            {11.2, 2.55, 0},   // route back (low lane)
    };

    // --- Sim Cockpit wiring --------------------------------------------------

    private final Map<String, Goal> goalsByName = Map.of(
            "Idle", IDLE, "Intake", INTAKE, "AimShoot", AIM_SHOOT);
    private final AtomicReference<String> pendingGoal = new AtomicReference<>(null);
    private final AtomicReference<Boolean> pendingEnable = new AtomicReference<>(null);
    private volatile boolean pendingReset = false;
    private volatile boolean pendingAuto = false;
    private final AtomicReference<String> pendingFault = new AtomicReference<>(null);
    private final AtomicReference<String> pendingToggle = new AtomicReference<>(null);

    // --- Toggleable features ---
    private boolean copilotOn = false, climbOn = false, opponentOn = false, ghostOn = false;
    private String copilotMode = null, copilotGoal = "";
    private Command copilotCmd;
    private double collectScore = 0, scoreScore = 0;

    // Opponent / defense bot
    private double oppX = 7.0, oppY = 2.0, oppVx = 0, oppVy = 0;
    private static final double OPP_R = 0.45;

    // Endgame climb
    private boolean climbing = false;

    // GhostReplay (real Catalyst): records the robot pose, replays it ghosted.
    private final GhostReplay ghost = new GhostReplay(
            () -> new Pose2d(poseX, poseY, new Rotation2d(heading)));
    private volatile String stateJson = "{}";
    private SimCockpit cockpit;

    // Generic, configurable mechanism cockpit (localhost:5806) — drives one of
    // every Catalyst mechanism kind, proving the sim adapts to any mechanism.
    private MechanismShowcase mechanismLab;
    private boolean autoEnabledOnce = false;
    /** The FMS game-specific message for this match: "R", "B", or empty before it is assessed. */
    private String gameData = "";
    private boolean lastAutoFlag = false;
    private boolean lastEnabledFlag = false;
    private double simClock = 0;
    private double matchStart = 0;

    public RobotContainer() {
        configureDefaults();
        // Real Catalyst safety watchdog: trip if too many concurrent faults.
        RobotSafety.configure(RobotSafety.Config.builder()
                .maxConcurrentErrors(1)
                .debounce(0.0)
                .onTrip(() -> AlertManager.getInstance().warning("RobotSafety", "Tripped — outputs safed"))
                .build());
        if (RobotBase.isSimulation()) {
            cockpit = new SimCockpit(
                    5805,
                    () -> stateJson,
                    pendingGoal::set,
                    pendingEnable::set,
                    (vx, vy, w) -> { cmdVx = vx; cmdVy = vy; cmdOmega = w; },
                    () -> pendingReset = true,
                    () -> pendingAuto = true,
                    pendingFault::set,
                    pendingToggle::set);
            mechanismLab = new MechanismShowcase();
        }
    }

    private void configureDefaults() {
        // The turret ALWAYS tracks the goal with SOTF lead (heading is fixed at
        // 0 for this holonomic demo). This is the headline behaviour: drive
        // around and the barrel stays on the virtual goal.
        turret.setDefaultCommand(turret.track(
                () -> solution,
                () -> Math.toDegrees(heading),       // chassis heading
                () -> Math.toDegrees(omega)));        // chassis yaw rate (exact FF while spinning)
        shooter.setDefaultCommand(shooter.runVoltage(0));
        hood.setDefaultCommand(hood.holdPosition());
        intake.setDefaultCommand(intake.runAtSpeed(0));
        climber.setDefaultCommand(climber.holdPosition());

        // Co-Pilot: the REAL Catalyst Strategist picks intake-vs-shoot each loop
        // by utility score. Each behavior's command just records the chosen mode;
        // the kinematic follower then drives accordingly. Scores publish to
        // /Catalyst/Behavior/CoPilot/Scores/*.
        Action collect = Action.named("Collect")
                .run(() -> Commands.run(() -> copilotMode = "Collect"))
                .build();
        Action scoreAct = Action.named("Score")
                .run(() -> Commands.run(() -> copilotMode = "Score"))
                .build();
        copilotCmd = Strategist.named("CoPilot")
                .add("Collect", collect, ctx -> collectScore)
                .add("Score", scoreAct, ctx -> scoreScore)
                .build();
    }

    /** Utility scores for the co-pilot (the real Strategist reads these each loop). */
    private void updateCopilotScores() {
        // Hysteresis: once collecting, keep collecting until the hopper is full;
        // once scoring, keep scoring until empty. That commits the robot to a
        // full drive across the field instead of dithering at the midpoint.
        boolean collecting = "Collect".equals(copilotMode);
        if (collecting) {
            collectScore = fuel >= 6 ? 2.0 : 12.0;
            scoreScore = fuel >= 6 ? 13.0 : 4.0;
        } else {
            scoreScore = fuel <= 1 ? 2.0 : 12.0;
            collectScore = fuel <= 1 ? 13.0 : 4.0;
        }
    }

    /** Called from {@link Robot#simulationPeriodic()} on the main thread. */
    public void simPeriodic() {
        if (!autoEnabledOnce) {
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().schedule(director.pursue(IDLE));
            autoEnabledOnce = true;
        }
        simClock += 0.02;

        // Pump the generic mechanism dashboard (drains its queued commands +
        // snapshots state on this, the main thread).
        if (mechanismLab != null) {
            mechanismLab.update();
        }

        Boolean en = pendingEnable.getAndSet(null);
        if (en != null) {
            DriverStationSim.setEnabled(en);
            DriverStationSim.notifyNewData();
        }
        if (pendingReset) {
            pendingReset = false;
            shots = 0; hits = 0; misses = 0; idealHits = 0; scored = 0;
            recentShots.clear();
            matchStart = simClock;
            autoRunning = false;
        }

        // Fault injection → real Catalyst AlertManager + RobotSafety watchdog.
        String fault = pendingFault.getAndSet(null);
        if (fault != null) {
            switch (fault) {
                case "overheat" -> AlertManager.getInstance().error("Turret", "Motor overheat 86C - thermal cutoff");
                case "stall" -> AlertManager.getInstance().error("Shooter", "Flywheel stall - 120A current spike");
                case "brownout" -> AlertManager.getInstance().warning("Power", "Brownout - battery sagged to 6.8V");
                case "clear" -> { AlertManager.getInstance().clearAll(); RobotSafety.reset(); }
                default -> { }
            }
        }
        // Feed the live fault counts to the watchdog every loop (real usage).
        RobotSafety.tick(AlertManager.getInstance().getErrors().size(),
                         AlertManager.getInstance().getWarnings().size());

        // Feature toggles from the cockpit.
        String tog = pendingToggle.getAndSet(null);
        if (tog != null) {
            switch (tog) {
                case "copilot" -> {
                    copilotOn = !copilotOn;
                    if (copilotOn) {
                        copilotMode = null; copilotGoal = "";
                        CommandScheduler.getInstance().schedule(copilotCmd);
                    } else if (copilotCmd != null) {
                        CommandScheduler.getInstance().cancel(copilotCmd);
                        copilotMode = null;
                    }
                }
                case "climb" -> {
                    climbOn = !climbOn;
                    if (!climbOn) { climbing = false; CommandScheduler.getInstance().schedule(climber.goTo("DOWN")); }
                }
                case "opponent" -> { opponentOn = !opponentOn; if (opponentOn) { oppX = 7.0; oppY = 2.0; oppVx = 0; oppVy = 0; } }
                case "ghost" -> {
                    ghostOn = !ghostOn;
                    CommandScheduler.getInstance().schedule(ghostOn ? ghost.startReplay("auto") : ghost.stopReplay());
                }
                default -> { }
            }
        }

        updateCopilotScores();
        updateOpponent();
        ghost.update();      // real GhostReplay: records while recording, advances replay

        String g = pendingGoal.getAndSet(null);
        if (g != null && !autoRunning) {
            Goal goal = goalsByName.get(g);
            if (goal != null) {
                CommandScheduler.getInstance().schedule(director.pursue(goal));
            }
        }

        runMatchCycle();

        // Autonomous routine: a planned waypoint path with intake/shoot actions.
        if (pendingAuto) {
            pendingAuto = false;
            startAuto();
        }
        if (autoRunning) {
            runAuto();
            if (simClock - autoT0 > AUTO_SECONDS) {
                autoRunning = false;
                cmdVx = 0; cmdVy = 0; cmdOmega = 0;
                CommandScheduler.getInstance().schedule(director.pursue(IDLE));
                CommandScheduler.getInstance().schedule(ghost.stopRecording());   // save the run
            }
        } else if (climbOn) {
            runClimb();                 // endgame Tower climb
        } else if (copilotOn) {
            runCoPilot();               // Strategist decides intake vs shoot
        }
        // else: manual drive (cmd from the cockpit)

        // Trail for path visualization.
        if (simClock - lastTrailT > 0.08) {
            trail.addLast(new double[]{poseX, poseY});
            while (trail.size() > 80) trail.removeFirst();
            lastTrailT = simClock;
        }

        // Slew the actual chassis velocity + yaw rate toward the cockpit command
        // (a real swerve accelerates over a few tenths, it doesn't step). Snappy
        // but smooth, so the turret/flywheel can track on the move.
        double aStep = 9.0 * 0.02;     // ~9 m/s^2 — realistic high-traction swerve accel
        double rStep = 12.0 * 0.02;    // rad/s^2 yaw accel
        velX += clamp(cmdVx - velX, -aStep, aStep);
        velY += clamp(cmdVy - velY, -aStep, aStep);
        omega += clamp(cmdOmega - omega, -rStep, rStep);

        heading += omega * 0.02;
        poseX += velX * 0.02;
        poseY += velY * 0.02;

        // Collision, through the library's own contact physics rather than an ad-hoc push-out. The
        // old version zeroed the velocity into a wall, which is restitution 0 for everything; this
        // bounces off polycarbonate differently from a padded field element, and reports the impact.
        Pose2d here = new Pose2d(poseX, poseY, new Rotation2d(heading));

        // Whichever branch resolves the contact, it only resolves the contact: everything after this
        // block runs either way. The heightmap branch used to return from simPeriodic() once it was
        // done, which silently switched off the solver, the shooting and the opponent for anyone who
        // had deployed the CAD map — the turret sat on whatever solution it last computed and never
        // tracked again. The showcase stopped being a showcase and nothing said so.
        if (HEIGHTMAP != null) {
            // The map already describes the perimeter, the hubs and everything else in the CAD, so the
            // rectangle-and-two-boxes model below is redundant here rather than complementary.
            for (int pass = 0; pass < 3; pass++) {
                var verdict = HEIGHTMAP.test(here, ROBOT_HALF_EXTENT, ROBOT_HALF_EXTENT, ROBOT_TALL, 0);
                groundHeight = verdict.groundHeight();
                if (!verdict.blocked()) break;
                poseX += verdict.normal().getX() * verdict.penetration();
                poseY += verdict.normal().getY() * verdict.penetration();
                double into = velX * verdict.normal().getX() + velY * verdict.normal().getY();
                if (into < 0) {
                    velX -= into * verdict.normal().getX();
                    velY -= into * verdict.normal().getY();
                    CatalystLog.log("Physics/Collision/Timestamp", simClock);
                    CatalystLog.log("Physics/Collision/MpsSq", Math.abs(into) / 0.02);
                    CatalystLog.log("Physics/Collision/Newtons", Math.abs(into) / 0.02 * ROBOT_MASS_KG);
                }
                here = new Pose2d(poseX, poseY, new Rotation2d(heading));
            }
            CatalystLog.log("Physics/GroundHeight", groundHeight);
        } else {
            var hit = COLLISION.contact(here, ROBOT_HALF_EXTENT, ROBOT_HALF_EXTENT);
            if (hit.isPresent()) {
                var contact = hit.get();
                poseX += contact.normal().getX() * contact.penetration();
                poseY += contact.normal().getY() * contact.penetration();

                var after = ContactResolver.resolveAgainstStatic(
                        new Translation3d(velX, velY, 0),
                        new Translation3d(contact.normal().getX(), contact.normal().getY(), 0),
                        ROBOT_MASS_KG, ContactMaterial.BUMPER, contact.material());

                if (after.resolved()) {
                    double impact = Math.hypot(after.velocityA().getX() - velX,
                                               after.velocityA().getY() - velY) / 0.02;
                    CatalystLog.log("Physics/Collision/Timestamp", simClock);
                    CatalystLog.log("Physics/Collision/MpsSq", impact);
                    CatalystLog.log("Physics/Collision/Newtons", impact * ROBOT_MASS_KG);
                }
                velX = after.velocityA().getX();
                velY = after.velocityA().getY();
            }

            // The scoring-relevant field elements the cockpit already knew about, kept for the drawing.
            for (double[] o : OBSTACLES) {
                double dx = poseX - o[0], dy = poseY - o[1];
                double d = Math.hypot(dx, dy);
                double minD = o[2] + ROBOT_RADIUS;
                if (d < minD && d > 1e-6) {
                    double nxn = dx / d, nyn = dy / d;
                    poseX = o[0] + nxn * minD;
                    poseY = o[1] + nyn * minD;
                    double into = velX * nxn + velY * nyn;   // velocity into the obstacle
                    if (into < 0) { velX -= into * nxn; velY -= into * nyn; }
                }
            }
        }

        // Opponent / defense bot collision (a moving obstacle you must dodge).
        if (opponentOn) {
            double dx = poseX - oppX, dy = poseY - oppY, d = Math.hypot(dx, dy),
                    minD = OPP_R + ROBOT_RADIUS;
            if (d < minD && d > 1e-6) {
                double nx = dx / d, ny = dy / d;
                poseX = oppX + nx * minD; poseY = oppY + ny * minD;
                double into = velX * nx + velY * ny;
                if (into < 0) { velX -= into * nx; velY -= into * ny; }
            }
        }

        // Recompute the SOTF solution from the live pose + ACTUAL field velocity.
        solution = solver.solve(
                new Pose2d(poseX, poseY, new Rotation2d(heading)),
                new ChassisSpeeds(velX, velY, omega));

        // Game logic. True SOTF: while Aim&Shoot is held and the flywheel is
        // spinning, we KEEP firing on the move — we do not wait for a perfect
        // "ready". Each shot's accuracy is decided by how well the turret and
        // flywheel were actually tracking at release; bad shots fly wide and
        // are shown as misses rather than blocking the trigger.
        firing = false;
        String active = director.activeGoalName();
        double shooterRpmNow = shooter.getVelocity() * 60.0;
        // Fuel is only collected near the center Fuel field / neutral zone — so a
        // robot (or the Co-Pilot) has to actually drive there to reload.
        if ("Intake".equals(active) && fuel < MAX_FUEL && poseX < 9.5
                && simClock - lastActionTs > 0.04) {
            fuel++;
            lastActionTs = simClock;
        } else if ("AimShoot".equals(active) && fuel > 0 && solution.feasible()
                && shooterRpmNow > 800 && !RobotSafety.isTripped()
                && simClock - lastActionTs > 0.12) {
            fireShot(shooterRpmNow);
            lastActionTs = simClock;
        }

        stateJson = buildStateJson();
        publishForConsole();
    }

    /**
     * Run a REBUILT match on a loop: 20 s auto, 140 s teleop, then a short reset before the next one.
     *
     * <p>Before this existed the simulation simply ran forever in teleop with no clock, which is
     * comfortable to develop against and useless for checking anything that depends on match time —
     * the shift schedule, the end game, a dashboard countdown. It drives {@link DriverStationSim} the
     * way a real field would, so robot code and any attached dashboard see a genuine match.
     *
     * <p>Segment boundaries are the ones in the 2026 Game Manual, Table 6-2. Teleop counts <em>down</em>
     * from 140, so each shift is named by the time remaining when it ends.
     */
    private void runMatchCycle() {
        double elapsed = simClock - matchStart;

        if (elapsed >= MATCH_SECONDS + RESET_SECONDS) {
            matchStart = simClock;
            elapsed = 0;
            gameData = "";
            hubActive = true;
        }

        boolean inAuto = elapsed < AUTO_SECONDS;
        boolean inMatch = elapsed < MATCH_SECONDS;
        double remaining = inAuto
                ? AUTO_SECONDS - elapsed
                : Math.max(0.0, MATCH_SECONDS - elapsed);

        if (inAuto != lastAutoFlag || inMatch != lastEnabledFlag) {
            lastAutoFlag = inAuto;
            lastEnabledFlag = inMatch;
            DriverStationSim.setAutonomous(inAuto);
            DriverStationSim.setEnabled(inMatch);
            DriverStationSim.notifyNewData();
        }

        // FMS assesses AUTO fuel and names the alliance whose hub sits out first, a few seconds into
        // teleop. A single character, empty until then — the format WPILib documents.
        if (gameData.isEmpty() && elapsed > AUTO_SECONDS + 3.0) {
            gameData = scored >= 4 ? "R" : "B";
            DriverStationSim.setGameSpecificMessage(gameData);
            DriverStationSim.notifyNewData();
        }

        hubActive = hubActiveNow(inAuto, remaining);
        CatalystLog.log("Match/TimeLeft", remaining);
    }

    /**
     * Whether <em>our</em> hub is scoring right now.
     *
     * <p>Both hubs are active through AUTO, the transition shift and END GAME. Across shifts 1-4 they
     * alternate, starting with the alliance named in the game data sitting out.
     */
    private boolean hubActiveNow(boolean inAuto, double remaining) {
        if (inAuto || gameData.isEmpty()) return true;
        for (int i = 0; i < SHIFT_ENDS.length; i++) {
            if (remaining > SHIFT_ENDS[i]) {
                if (i == 0 || i == SHIFT_ENDS.length - 1) return true;   // transition shift, end game
                int shift = i - 1;                                        // 0-based shift 1..4
                boolean weSitOutFirst = gameData.startsWith("R");         // this demo runs red
                return weSitOutFirst ? shift % 2 == 1 : shift % 2 == 0;
            }
        }
        return true;
    }

    /**
     * Publish the handful of keys a driver station dashboard binds to.
     *
     * <p>This is the whole robot-side contract for
     * <a href="https://github.com/TomAs-1226/CatalystConsole">Catalyst Console</a>, and it is short on
     * purpose: alerts and loop time already go out through {@link AlertManager} and
     * {@link LoopMonitor} without being asked, so all that is left is the pose, the game state the
     * dashboard cannot know, and the two values WPILib does not put on NetworkTables for you.
     *
     * <p>The pose here is the cockpit's simulated one rather than a {@code PhysicsCore} estimate —
     * this showcase has no drivetrain to estimate from. On a real robot the same key comes out of
     * Physics Core for free.
     */
    private void publishForConsole() {
        loop.record();
        CatalystLog.log("Physics/PoseArray", new double[] { poseX, poseY, heading });

        // Whether our hub is scoring right now, and how long this segment has left. A dashboard can
        // work both out from the rules and the FMS game data, but the robot already knows, so it says.
        double elapsed = simClock - matchStart;
        double remaining = elapsed < AUTO_SECONDS
                ? AUTO_SECONDS - elapsed
                : Math.max(0.0, MATCH_SECONDS - elapsed);
        double untilChange = remaining;
        if (elapsed >= AUTO_SECONDS) {
            for (double end : SHIFT_ENDS) {
                if (remaining > end) { untilChange = remaining - end; break; }
            }
        }
        CatalystLog.log("Game/TowerActive", hubActive);
        CatalystLog.log("Game/TowerSeconds", untilChange);

        CatalystLog.log("Status/CanUtilization", RobotController.getCANStatus().percentBusUtilization);
        CatalystLog.log("Status/BatteryVolts", RobotController.getBatteryVoltage());
    }

    private String buildStateJson() {
        AimingSolver.Solution s = solution;
        Translation2d vg = s.virtualGoal();
        boolean enabled = edu.wpi.first.wpilibj.DriverStation.isEnabled();
        StringBuilder sb = new StringBuilder(700);
        sb.append('{');
        sb.append("\"enabled\":").append(enabled).append(',');
        sb.append("\"mode\":\"Teleop\",");
        sb.append("\"field\":{\"w\":").append(f(fieldLength())).append(",\"h\":").append(f(fieldWidth())).append("},");
        // The cockpit draws a square of side 2*robotR, so this is the half-extent, not the disc radius.
        sb.append("\"robotR\":").append(ROBOT_HALF_EXTENT).append(',');
        sb.append("\"matchTime\":").append(f(Math.max(0.0, MATCH_SECONDS - (simClock - matchStart)))).append(',');
        sb.append("\"obstacles\":[");
        for (int i = 0; i < OBSTACLES.length; i++) {
            if (i > 0) sb.append(',');
            sb.append("{\"x\":").append(f(OBSTACLES[i][0])).append(",\"y\":").append(f(OBSTACLES[i][1]))
              .append(",\"r\":").append(f(OBSTACLES[i][2])).append('}');
        }
        sb.append("],");
        sb.append("\"fieldEls\":{\"centerX\":").append(f(fieldLength() / 2.0)).append(",\"allianceZone\":").append(f(ALLIANCE_ZONE))
          .append(",\"redHub\":{\"x\":").append(f(RED_HUB.getX())).append(",\"y\":").append(f(RED_HUB.getY())).append(",\"r\":").append(f(HUB_R)).append('}')
          .append(",\"blueHub\":{\"x\":").append(f(BLUE_HUB.getX())).append(",\"y\":").append(f(BLUE_HUB.getY())).append(",\"r\":").append(f(HUB_R)).append('}')
          .append(",\"redTower\":{\"x\":").append(f(RED_TOWER.getX())).append(",\"y\":").append(f(RED_TOWER.getY())).append(",\"r\":").append(f(TOWER_R)).append('}')
          .append(",\"blueTower\":{\"x\":").append(f(BLUE_TOWER.getX())).append(",\"y\":").append(f(BLUE_TOWER.getY())).append(",\"r\":").append(f(TOWER_R)).append('}')
          .append("},");
        sb.append("\"goalPt\":{\"x\":").append(f(GOAL.getX())).append(",\"y\":").append(f(GOAL.getY())).append("},");
        sb.append("\"virtualGoal\":{\"x\":").append(f(vg.getX())).append(",\"y\":").append(f(vg.getY())).append("},");
        sb.append("\"robot\":{\"x\":").append(f(poseX)).append(",\"y\":").append(f(poseY))
          .append(",\"heading\":").append(f(heading))
          .append(",\"vx\":").append(f(velX)).append(",\"vy\":").append(f(velY))
          .append(",\"omega\":").append(f(omega)).append("},");
        sb.append("\"turret\":{\"field\":").append(f(turret.getAngle() + Math.toDegrees(heading)))
          .append(",\"aimField\":").append(f(s.turretFieldAngleDeg()))
          .append(",\"atSetpoint\":").append(turret.atSetpoint()).append("},");
        sb.append("\"shooter\":{\"rpm\":").append(f(shooter.getVelocity() * 60.0))
          .append(",\"target\":").append(f(shooter.getSetpoint() * 60.0))
          .append(",\"atSpeed\":").append(shooter.atSpeed()).append("},");
        sb.append("\"hood\":{\"deg\":").append(f(hood.getAngle()))
          .append(",\"target\":").append(f(hood.getSetpoint())).append("},");
        sb.append("\"aim\":{\"distance\":").append(f(s.distanceMeters()))
          .append(",\"shotTime\":").append(f(s.shotTimeSeconds()))
          .append(",\"rateDps\":").append(f(s.turretFieldRateDps()))
          .append(",\"solRpm\":").append(f(s.shooterRpm()))
          .append(",\"solHood\":").append(f(s.hoodDegrees()))
          .append(",\"feasible\":").append(s.feasible())
          .append(",\"maxRange\":4.0},");
        sb.append("\"fuel\":").append(fuel).append(',');
        sb.append("\"shots\":").append(shots).append(',');
        sb.append("\"hits\":").append(hits).append(',');
        sb.append("\"misses\":").append(misses).append(',');
        sb.append("\"idealHits\":").append(idealHits).append(',');
        sb.append("\"firing\":").append(firing).append(',');
        sb.append("\"recentShots\":[");
        boolean firstShot = true;
        for (ShotRec r : recentShots) {
            if (!firstShot) sb.append(',');
            firstShot = false;
            sb.append("{\"id\":").append(r.id())
              .append(",\"x0\":").append(f(r.x0())).append(",\"y0\":").append(f(r.y0()))
              .append(",\"lx\":").append(f(r.lx())).append(",\"ly\":").append(f(r.ly()))
              .append(",\"hit\":").append(r.hit()).append(",\"tof\":").append(f(r.tof())).append('}');
        }
        sb.append("],");
        sb.append("\"goal\":{\"active\":\"").append(esc(director.activeGoalName()))
          .append("\",\"ready\":").append(director.isReady()).append("},");
        sb.append("\"scored\":").append(scored).append(',');
        sb.append("\"match\":{\"phase\":\"")
          .append((autoRunning || (simClock - matchStart) < AUTO_SECONDS) ? "AUTO" : "TELEOP")
          .append("\",\"hubActive\":").append(hubActive).append("},");
        sb.append("\"safety\":{\"tripped\":").append(RobotSafety.isTripped())
          .append(",\"reason\":\"").append(esc(RobotSafety.reason())).append("\"},");
        Pose2d gp = ghost.currentGhostPose();
        double gx = gp != null ? gp.getX() : 0, gy = gp != null ? gp.getY() : 0,
               gh = gp != null ? gp.getRotation().getRadians() : 0;
        sb.append("\"copilot\":{\"on\":").append(copilotOn).append(",\"mode\":\"").append(esc(copilotMode == null ? "" : copilotMode))
          .append("\",\"collect\":").append(f(collectScore)).append(",\"score\":").append(f(scoreScore)).append("},");
        sb.append("\"climb\":{\"on\":").append(climbOn).append(",\"pct\":").append(f(climber.getPosition() / 0.55))
          .append(",\"climbing\":").append(climbing).append("},");
        sb.append("\"opp\":{\"on\":").append(opponentOn).append(",\"x\":").append(f(oppX)).append(",\"y\":").append(f(oppY)).append("},");
        sb.append("\"ghost\":{\"on\":").append(ghostOn).append(",\"replaying\":").append(ghost.isReplaying())
          .append(",\"x\":").append(f(gx)).append(",\"y\":").append(f(gy)).append(",\"hdg\":").append(f(gh)).append("},");
        sb.append("\"auto\":{\"running\":").append(autoRunning)
          .append(",\"idx\":").append(autoIdx).append(",\"scored\":").append(autoScored).append(",\"plan\":[");
        for (int i = 0; i < AUTO_PLAN.length; i++) {
            if (i > 0) sb.append(',');
            sb.append("{\"x\":").append(f(AUTO_PLAN[i][0])).append(",\"y\":").append(f(AUTO_PLAN[i][1]))
              .append(",\"mode\":").append((int) AUTO_PLAN[i][2]).append('}');
        }
        sb.append("],\"trail\":[");
        boolean ft = true;
        for (double[] t : trail) {
            if (!ft) sb.append(',');
            ft = false;
            sb.append("{\"x\":").append(f(t[0])).append(",\"y\":").append(f(t[1])).append('}');
        }
        sb.append("]},");
        sb.append("\"errors\":[").append(jsonList(AlertManager.getInstance().getErrors())).append("],");
        sb.append("\"warnings\":[").append(jsonList(AlertManager.getInstance().getWarnings())).append("]");
        sb.append('}');
        return sb.toString();
    }

    /** Begin the autonomous routine: reset to the starting line and run the plan. */
    private void startAuto() {
        autoRunning = true;
        autoT0 = simClock;
        autoIdx = 0;
        autoScored = 0;
        autoWpReachedT = -1;
        poseX = START_X; poseY = START_Y; heading = 0;
        velX = 0; velY = 0; omega = 0;
        fuel = 12;                                  // preload
        shots = 0; hits = 0; misses = 0; idealHits = 0; scored = 0;
        recentShots.clear();
        trail.clear();
        matchStart = simClock;                      // match clock starts at auto
        CommandScheduler.getInstance().schedule(director.pursue(INTAKE));
        CommandScheduler.getInstance().schedule(ghost.startRecording("auto"));   // record for replay
    }

    /** One step of the planned autonomous path: drive to the waypoint, then act. */
    private void runAuto() {
        if (autoIdx >= AUTO_PLAN.length) autoIdx = 0;   // loop the plan to fill 20 s
        double[] wp = AUTO_PLAN[autoIdx];
        double dx = wp[0] - poseX, dy = wp[1] - poseY, d = Math.hypot(dx, dy);
        int mode = (int) wp[2];

        if (d > 0.18 && autoWpReachedT < 0) {
            double s = Math.min(3.0, d * 4.0);          // P-control, ease into the point
            cmdVx = dx / d * s;
            cmdVy = dy / d * s;
            // Turn the chassis to face its travel direction so the intake (front)
            // leads into the Fuel when collecting.
            double hErr = wrapPi(Math.atan2(dy, dx) - heading);
            cmdOmega = clamp(hErr * 5.0, -3.0, 3.0);
        } else {
            cmdVx = 0; cmdVy = 0; cmdOmega = 0;
            if (autoWpReachedT < 0) {
                autoWpReachedT = simClock;
                if (mode == 1) CommandScheduler.getInstance().schedule(director.pursue(INTAKE));
                else if (mode == 2) CommandScheduler.getInstance().schedule(director.pursue(AIM_SHOOT));
            }
            double dwell = (mode == 0) ? 0.1 : 1.1;     // linger to intake / shoot
            if (simClock - autoWpReachedT > dwell) {
                autoWpReachedT = -1;
                autoIdx++;
            }
        }
    }

    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /** Drive the kinematic chassis toward a field point, facing the travel direction. */
    private void driveToward(double tx, double ty) {
        double dx = tx - poseX, dy = ty - poseY, d = Math.hypot(dx, dy);
        if (d > 0.15) {
            double s = Math.min(3.0, d * 4.0);
            cmdVx = dx / d * s;
            cmdVy = dy / d * s;
            cmdOmega = clamp(wrapPi(Math.atan2(dy, dx) - heading) * 5.0, -3.0, 3.0);
        } else {
            cmdVx = 0; cmdVy = 0; cmdOmega = 0;
        }
    }

    /** Co-Pilot: drive per the Strategist's chosen mode (Collect vs Score). */
    private void runCoPilot() {
        if (copilotMode == null) return;
        if (copilotMode.equals("Collect")) {
            scheduleCopilotGoal(INTAKE, "Intake");
            driveToward(8.2, clamp(poseY, 2.6, 5.6));    // dip into the center Fuel field
        } else {
            // Drive to the scoring zone holding fire, then shoot once parked —
            // so it doesn't dump fuel en route and turn around half-empty.
            double tx = 12.9, ty = 5.0;
            if (Math.hypot(tx - poseX, ty - poseY) > 0.3) {
                scheduleCopilotGoal(IDLE, "Idle");
                driveToward(tx, ty);
            } else {
                scheduleCopilotGoal(AIM_SHOOT, "AimShoot");
                cmdVx = 0; cmdVy = 0; cmdOmega = 0;
            }
        }
    }

    private void scheduleCopilotGoal(Goal g, String name) {
        if (!name.equals(copilotGoal)) {
            copilotGoal = name;
            CommandScheduler.getInstance().schedule(director.pursue(g));
        }
    }

    /** Endgame Tower climb: drive to the tower, then raise the climber. */
    private void runClimb() {
        double tx = 14.75, ty = 4.10;                    // standoff in front of the blue tower
        double d = Math.hypot(tx - poseX, ty - poseY);
        if (d > 0.22) {
            driveToward(tx, ty);
            climbing = false;
        } else {
            cmdVx = 0; cmdVy = 0; cmdOmega = 0;
            if (!climbing) {
                climbing = true;
                CommandScheduler.getInstance().schedule(climber.goTo("UP"));
            }
        }
    }

    /** Opponent / defense bot AI: slide into the lane between us and the hub. */
    private void updateOpponent() {
        if (!opponentOn) return;
        double bx = BLUE_HUB.getX() + (poseX - BLUE_HUB.getX()) * 0.45;
        double by = BLUE_HUB.getY() + (poseY - BLUE_HUB.getY()) * 0.45;
        double dx = bx - oppX, dy = by - oppY, d = Math.hypot(dx, dy);
        double sp = 2.2, st = 6.0 * 0.02;
        double tvx = d > 0.1 ? dx / d * sp : 0, tvy = d > 0.1 ? dy / d * sp : 0;
        oppVx += clamp(tvx - oppVx, -st, st);
        oppVy += clamp(tvy - oppVy, -st, st);
        oppX = clamp(oppX + oppVx * 0.02, OPP_R, fieldLength() - OPP_R);
        oppY = clamp(oppY + oppVy * 0.02, OPP_R, fieldWidth() - OPP_R);
    }

    /**
     * Model one shot leaving the turret. The ball is launched along the turret's
     * actual field bearing at a speed set by the flywheel, and — crucially — it
     * <b>inherits the robot's field velocity</b>. That inheritance is exactly
     * why Shoot-On-The-Fly works: aim at the lead point, and the robot's motion
     * carries the ball onto the real goal. If the turret was lagging or the
     * flywheel was off-speed at release, the net trajectory misses, and we draw
     * it missing.
     */
    private void fireShot(double shooterRpmNow) {
        AimingSolver.Solution s = solution;
        double tof = Math.max(0.2, s.shotTimeSeconds());
        // Field bearing of the barrel = turret robot-relative angle + chassis heading.
        double angRad = Math.toRadians(turret.getAngle()) + heading;
        Translation2d aim = s.virtualGoal();
        double dAim = Math.hypot(aim.getX() - poseX, aim.getY() - poseY);
        double targetRpm = Math.max(1.0, s.shooterRpm());
        double rpmRatio = clamp(shooterRpmNow / targetRpm, 0.3, 1.8);
        double groundSpeed = (dAim / tof) * rpmRatio;             // m/s toward the aim point

        double vbx = Math.cos(angRad) * groundSpeed + velX;       // + inherited robot velocity
        double vby = Math.sin(angRad) * groundSpeed + velY;
        double lx = poseX + vbx * tof;
        double ly = poseY + vby * tof;
        boolean hit = Math.hypot(lx - GOAL.getX(), ly - GOAL.getY()) < 0.8;

        // Ideal shot: if the turret were exactly on the solved bearing and the
        // flywheel exactly at the target RPM. With the math correct this lands
        // on the goal regardless of motion — so idealHits tracks math quality,
        // while (hits) tracks how well the real mechanisms are keeping up.
        double iAng = Math.toRadians(s.turretFieldAngleDeg());
        double iSpeed = dAim / tof;
        double ilx = poseX + (Math.cos(iAng) * iSpeed + velX) * tof;
        double ily = poseY + (Math.sin(iAng) * iSpeed + velY) * tof;
        if (Math.hypot(ilx - GOAL.getX(), ily - GOAL.getY()) < 0.8) idealHits++;

        fuel--;
        shots++;
        if (hit) {
            hits++;
            if (hubActive) { scored++; if (autoRunning) autoScored++; }   // only scores when active
        } else {
            misses++;
        }
        recentShots.addLast(new ShotRec(shots, poseX, poseY, lx, ly, hit, tof));
        while (recentShots.size() > 12) recentShots.removeFirst();
        firing = true;
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
                director.pursue(INTAKE).withTimeout(1.5),
                director.pursue(AIM_SHOOT).withTimeout(3.0),
                director.pursue(IDLE)).withName("Auto.Demo");
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String f(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "0.0";
        return String.format("%.3f", v);
    }

    private static String esc(String s) {
        return s == null ? "" : s.replace("\\", "\\\\").replace("\"", "\\\"");
    }

    private static String jsonList(List<String> items) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < items.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append('"').append(esc(items.get(i))).append('"');
        }
        return sb.toString();
    }
}
