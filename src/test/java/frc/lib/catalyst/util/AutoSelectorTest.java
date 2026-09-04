package frc.lib.catalyst.util;

import org.junit.jupiter.api.Test;
import org.wpilib.smartdashboard.SendableChooser;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The auto chooser, which this branch shipped without for no good reason.
 *
 * <p>{@code AutoSelector} was ported to {@code org.wpilib.tunable.Selectable} on the belief that
 * {@code SendableChooser} had been removed in 2027, then dropped from the source set entirely when
 * {@code org.wpilib.tunables} turned out to be missing from the released alpha-6. Only the second
 * half was true. Checked against the shipped jars: {@code SendableChooser}, {@code SmartDashboard},
 * {@code Sendable} and {@code SendableBuilder} are all present in {@code wpilibj-java}; the removal
 * claim came from the development snapshot, a different build that shares the alpha-6 name.
 *
 * <p>The cost was that a team on this branch had no Catalyst auto chooser at all — and the class
 * being excluded from the build meant no test could notice, because there was nothing to compile
 * against. That is the specific hazard of fixing a compile error by deleting the file from the
 * build: the tests go quiet along with it.
 */
class AutoSelectorTest {

    @Test
    void thereIsAlwaysASafeDefault() {
        // A selector whose default does something is how a robot moves before anyone told it to.
        AutoSelector autos = new AutoSelector("AutoSelectorTest/Default");

        assertEquals("Do Nothing", autos.getSelectedName());
        assertNotNull(autos.getSelected(), "the default must still produce a runnable command");
    }

    @Test
    void theChooserIsASendableChooserAgain() {
        // Not a cosmetic assertion: getChooser()'s return type was recorded in the 2027 notes as
        // the one signature Catalyst was forced to break. It was not, so a team migrating from 1.x
        // has nothing to change here, and this pins that.
        AutoSelector autos = new AutoSelector("AutoSelectorTest/Type");
        SendableChooser<String> chooser = autos.getChooser();

        assertNotNull(chooser);
        assertEquals("Do Nothing", chooser.getSelected());
    }

    @Test
    void aCustomAutoIsSelectableAndRunnable() {
        AutoSelector autos = new AutoSelector("AutoSelectorTest/Custom")
                .addCustom("Taxi", frc.lib.catalyst.command.Commands::none);

        autos.getChooser().setDefaultOption("Taxi", "Taxi");

        assertEquals("Taxi", autos.getSelectedName());
        assertNotNull(autos.getSelected());
    }

    @Test
    void anUnknownSelectionFallsBackRatherThanThrowing() {
        // The dashboard and the code can disagree — a renamed auto, a stale saved selection. The
        // robot must still be enable-able, and it must not run something arbitrary.
        AutoSelector autos = new AutoSelector("AutoSelectorTest/Unknown");
        autos.getChooser().addOption("Ghost", "Ghost");
        autos.getChooser().setDefaultOption("Ghost", "Ghost");

        assertNotNull(autos.getSelected(),
                "a selection with no registered command must still yield a command, not null");
    }

    @Test
    void theClassIsActuallyInTheBuild() {
        // The point of the whole exercise. This test cannot compile if AutoSelector is excluded from
        // the source set again, which is exactly the failure mode that hid the original mistake.
        assertTrue(AutoSelector.class.getName().endsWith("AutoSelector"));
    }
}
