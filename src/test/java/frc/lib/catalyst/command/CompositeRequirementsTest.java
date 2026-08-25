package frc.lib.catalyst.command;

import org.junit.jupiter.api.Test;

import org.wpilib.command3.Mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A composite has to declare what its children control.
 *
 * <p>{@code sequence}, {@code parallel} and {@code race} get this from v3's group builders.
 * {@code either} and {@code repeatingSequence} are hand-built, and both declared nothing — which is
 * not cosmetic in either case.
 *
 * <p>A repeating sequence with no requirements releases its mechanisms at every cycle boundary, so
 * the drivetrain's default command takes a full loop of stick input back between iterations. On
 * {@code Autopilot}, whose own documentation promises the driver's steering stays suspended, that
 * is a visible twitch of the swerve every time an action completes.
 *
 * <p>Requirements hidden inside an {@code either} defeat v3's build-time conflict check, which is
 * the check that exists to stop two commands driving one motor. It fires for
 * {@code parallel(a, b)} and used to sail straight past {@code parallel(either(a), either(b))}.
 */
class CompositeRequirementsTest {

    private record Mech(String name) implements Mechanism {
        @Override public String getName() {
            return name;
        }
    }

    private static CatalystCommand using(Mechanism m) {
        return Commands.run(() -> { }, m).withName("hold " + m.getName());
    }

    // --- what each composite declares ----------------------------------------

    @Test
    void eitherDeclaresBothBranches() {
        // Both, because the selector is not read until start and either branch may be the one that
        // runs. Reserving only the one that happens to be chosen is not knowable at schedule time.
        Mech arm = new Mech("arm");
        Mech wrist = new Mech("wrist");

        var requirements = Commands.either(using(arm), using(wrist), () -> true).requirements();

        assertTrue(requirements.contains(arm), "the true branch");
        assertTrue(requirements.contains(wrist), "the false branch");
    }

    @Test
    void repeatingSequenceDeclaresEveryStep() {
        Mech arm = new Mech("arm");
        Mech drive = new Mech("drive");

        var requirements = Commands.repeatingSequence(using(arm), using(drive)).requirements();

        assertEquals(2, requirements.size());
        assertTrue(requirements.contains(arm));
        assertTrue(requirements.contains(drive));
    }

    @Test
    void oneMechanismUsedTwiceIsDeclaredOnce() {
        Mech arm = new Mech("arm");
        assertEquals(1, Commands.repeatingSequence(using(arm), using(arm)).requirements().size());
    }

    @Test
    void aCompositeOfCommandsThatNeedNothingNeedsNothing() {
        assertTrue(Commands.repeatingSequence(Commands.none(), Commands.none())
                .requirements().isEmpty());
    }

    // --- the check those requirements exist to feed ---------------------------

    @Test
    void twoCompositesFightingOverOneMechanismAreRejectedAtConstruction() {
        // The whole point. v3 refuses parallel(a, b) when both need one mechanism, and the wrapping
        // must not hide it - a conflict found at construction is a compile-and-run away from being
        // fixed, and the same conflict found on the field is one branch silently cancelling the
        // other's entire composition mid-match.
        Mech arm = new Mech("arm");

        assertThrows(IllegalArgumentException.class,
                () -> Commands.parallel(
                        Commands.either(using(arm), Commands.none(), () -> true),
                        Commands.either(using(arm), Commands.none(), () -> false)),
                "two eithers both reaching for the arm is a conflict, wrapped or not");
    }

    @Test
    void compositesOverDifferentMechanismsStillCompose() {
        // The fix must not make legitimate compositions throw.
        Mech arm = new Mech("arm");
        Mech drive = new Mech("drive");

        var both = Commands.parallel(
                Commands.either(using(arm), Commands.none(), () -> true),
                Commands.repeatingSequence(using(drive)));

        assertTrue(both.requirements().contains(arm));
        assertTrue(both.requirements().contains(drive));
    }
}
