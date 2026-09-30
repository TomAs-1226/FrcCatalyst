package frc.lib.catalyst.fieldlab;

import frc.lib.catalyst.opmode.CatalystOpMode;

/**
 * A {@link TestCampaign} as a Driver Station Utility op mode.
 *
 * <h2>Using it</h2>
 *
 * <pre>{@code
 * @Utility(name = "Field Lab", group = "Data",
 *          description = "The field-day measurement campaign; needs an operator with a tape measure")
 * public class FieldLab extends FieldLabOpMode {
 *     public FieldLab() {
 *         super(Campaigns.fieldDay(Robot.ROBOT.drive()));
 *     }
 * }
 * }</pre>
 *
 * <p>and it appears on the Driver Station under <b>Data</b>, ready to select. Enable to run; disable
 * to pause. Nothing else is needed.
 *
 * <h2>Why a Utility op mode and not a dashboard button</h2>
 *
 * <p>Because a campaign drives the robot, and the one invariant every Catalyst diagnostic keeps is
 * that a dashboard never does. Catalyst publishes no command triggers over NetworkTables:
 * {@code SystemCheck}, {@code Zero wheels}, {@code Find motor}, the wheel-radius calibration and SysId
 * are all Utility op modes, and Catalyst Console and Catalyst Tab show their <em>results</em> and name
 * the op mode to run. A campaign is the largest of those routines and belongs in the same place. What
 * the dashboards do get is the prompt and the progress, which is what an operator standing on the
 * field actually needs.
 *
 * <h2>Disable, enable, resume</h2>
 *
 * <p>Disabling stops the campaign's command, as it stops every command. Enabling again starts this op
 * mode over, and the runner reads its checkpoint and carries on from the next unfinished procedure —
 * so the ordinary rhythm of a field session (disable, swap a battery, enable) is also the way the
 * campaign is meant to be paused. Nothing banked is re-run and nothing measured is re-asked.
 *
 * <p>While disabled it keeps publishing, because the two moments an operator is most likely to be
 * looking at the dashboard — held at a battery gate, or being asked for a measurement — are moments
 * when no command is running.
 *
 * @since 2.0.0
 */
public abstract class FieldLabOpMode extends CatalystOpMode {

    private final CampaignRunner runner;

    /** An op mode for this campaign, with the default checkpoint and the robot's battery voltage. */
    protected FieldLabOpMode(TestCampaign campaign) {
        this(new CampaignRunner(campaign));
    }

    /** An op mode for a runner built by hand — an explicit checkpoint path, or an injected battery. */
    protected FieldLabOpMode(CampaignRunner runner) {
        this.runner = runner;
    }

    /** The runner, for a subclass that wants to read the state or the measurements. */
    protected final CampaignRunner runner() {
        return runner;
    }

    @Override
    protected void onStart() {
        run(runner.command());
    }

    @Override
    protected void onPeriodic() {
        runner.publish();
    }

    @Override
    protected void onDisabledPeriodic() {
        runner.publish();
    }
}
