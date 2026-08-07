package frc.lib.catalyst.identity;

import java.util.Optional;

/**
 * Which build of Catalyst is actually running.
 *
 * <p>Before 1.10.0 the answer was "nobody knows". The version lived in {@code build.gradle} and
 * nowhere else: the jar manifest carried only {@code Manifest-Version}, so
 * {@code Package.getImplementationVersion()} returned {@code null}, and a team debugging a match
 * had no way to confirm which library their robot was running. The build now compiles the answer
 * in, which is why it survives being shaded into a robot's fat jar.
 *
 * <p>The version is always known. The git stamp is not: a build made from a source archive with no
 * repository in it has no commit to name, and that case reports empty rather than inventing a
 * placeholder. {@link RobotIdentity} publishes each of these only when it is present.
 *
 * <pre>{@code
 * CatalystVersion.version();    // "1.10.0"
 * CatalystVersion.describe();   // "1.10.0 (6e02513, dirty)"
 * }</pre>
 *
 * @since 1.10.0
 */
public final class CatalystVersion {

    private CatalystVersion() {}

    /** The library version, e.g. {@code "1.10.0"}. Compiled in, so always known. */
    public static String version() {
        return BuildConstants.VERSION;
    }

    /** Short git sha this jar was built from. Empty when the build had no repository. */
    public static Optional<String> gitSha() {
        return SpecSheet.text(BuildConstants.GIT_SHA);
    }

    /**
     * Whether the working tree carried uncommitted changes when this jar was built.
     *
     * <p>Empty whenever {@link #gitSha()} is: with no repository there is no tree to have been
     * dirty, and reporting a confident {@code false} would claim the build matched a commit that
     * was never identified.
     */
    public static Optional<Boolean> isDirty() {
        return gitSha().map(sha -> BuildConstants.GIT_DIRTY);
    }

    /** ISO-8601 committer date of the commit this jar was built from, when there was one. */
    public static Optional<String> commitTime() {
        return SpecSheet.text(BuildConstants.GIT_TIME);
    }

    /**
     * One line naming the build: {@code "1.10.0 (6e02513, dirty)"}, or just {@code "1.10.0"} when
     * there is no commit to name.
     */
    public static String describe() {
        return gitSha()
                .map(sha -> version() + " (" + sha + (BuildConstants.GIT_DIRTY ? ", dirty)" : ")"))
                .orElseGet(CatalystVersion::version);
    }
}
