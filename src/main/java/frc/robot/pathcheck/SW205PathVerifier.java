package frc.robot.pathcheck;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Compares the student's PathPlanner paths and SW205 auto against the answer key
 * embedded in this file. Students never see the .path files, only this check.
 * Run at robot startup; results are printed to the console.
 */
public final class SW205PathVerifier {
    public static final double START_END_POS_TOL_M = 0.10;
    public static final double ROUTE_POS_TOL_M = 0.01;
    public static final double ROTATION_TOL_DEG = 0.1;
    public static final double TIME_ABS_TOL_S = 0.6;
    public static final double TIME_REL_TOL = 0.30;
    public static final double MARKER_POS_TOL = 0.15;
    public static final boolean REQUIRE_EVENT_MARKERS = false;

    private static final int ROUTE_SAMPLES = 40;

    private record Check(boolean pass, String label, String detail) {}

    record PoseError(double posErrM, double rotErrDeg) {}

    record RouteError(double maxPosErrM, double maxRotErrDeg, double worstTimeS) {}

    private SW205PathVerifier() {}

    public static boolean verifyAllAndPrint() {
        System.out.println();
        printBanner("SW205 PATH VERIFIER - checking your paths against the lesson targets");
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.out.println("[SKIP] Could not load PathPlanner settings.json: " + e.getMessage());
            return true;
        }

        boolean allPass = true;
        allPass &= verifyPath("StartToPreloadF", referenceStartToPreloadF(), config);
        allPass &= verifyPath("ReefFToStation", referenceReefFToStation(), config);
        allPass &= verifyPath("ReefDToStation", referenceReefDToStation(), config);
        allPass &= verifyAutoSequence();

        printBanner(allPass
                ? "RESULT: PASS - every path reaches the right place at the right time"
                : "RESULT: FAIL - fix the [FAIL] items above in PathPlanner, then re-run");
        System.out.println();
        return allPass;
    }

    private static boolean verifyPath(String name, PathPlannerPath reference, RobotConfig config) {
        List<Check> checks = new ArrayList<>();
        PathPlannerPath student;
        try {
            student = PathPlannerPath.fromPathFile(name);
        } catch (Exception e) {
            checks.add(new Check(false, "file", "could not load pathplanner/paths/" + name
                    + ".path (" + e.getClass().getSimpleName() + ") - create it per the lesson video"));
            return report(name, checks);
        }

        Optional<PathPlannerTrajectory> refTraj = generateIdeal(reference, config);
        Optional<PathPlannerTrajectory> stuTraj = generateIdeal(student, config);
        if (refTraj.isEmpty() || stuTraj.isEmpty()) {
            checks.add(new Check(false, "trajectory", "failed to generate trajectory from this path"));
            return report(name, checks);
        }
        var ref = refTraj.get();
        var stu = stuTraj.get();

        PoseError startErr = poseError(ref.getInitialState().pose, stu.getInitialState().pose);
        checks.add(new Check(
                startErr.posErrM <= START_END_POS_TOL_M && startErr.rotErrDeg <= ROTATION_TOL_DEG,
                "start pose",
                startErr.posErrM <= START_END_POS_TOL_M && startErr.rotErrDeg <= ROTATION_TOL_DEG
                        ? describePose(startErr)
                        : describePose(startErr) + " - first waypoint is wrong"));

        PoseError endErr = poseError(ref.getEndState().pose, stu.getEndState().pose);
        checks.add(new Check(
                endErr.posErrM <= START_END_POS_TOL_M && endErr.rotErrDeg <= ROTATION_TOL_DEG,
                "end pose",
                endErr.posErrM <= START_END_POS_TOL_M && endErr.rotErrDeg <= ROTATION_TOL_DEG
                        ? describePose(endErr)
                        : describePose(endErr) + " - last waypoint or goal rotation is wrong"));

        RouteError route = routeError(ref, stu);
        boolean routeOk = route.maxPosErrM <= ROUTE_POS_TOL_M && route.maxRotErrDeg <= ROTATION_TOL_DEG;
        checks.add(new Check(routeOk, "route vs time",
                String.format(Locale.US, "worst deviation %.2f m / %.0f deg at t=%.2f s%s",
                        route.maxPosErrM, route.maxRotErrDeg, route.worstTimeS,
                        routeOk ? "" : " - mid-path shape strays off target")));

        double refTime = ref.getTotalTimeSeconds();
        double stuTime = stu.getTotalTimeSeconds();
        checks.add(new Check(durationWithinTolerance(refTime, stuTime), "travel time",
                String.format(Locale.US, "%.2f s vs target %.2f s (%+.2f s)", stuTime, refTime,
                        stuTime - refTime)));

        addMarkerChecks(reference, student, checks);

        return report(name, checks);
    }

    private static void addMarkerChecks(PathPlannerPath reference, PathPlannerPath student,
            List<Check> checks) {
        for (EventMarker expected : reference.getEventMarkers()) {
            var match = student.getEventMarkers().stream()
                    .filter(m -> m.triggerName().equals(expected.triggerName()))
                    .findFirst();
            if (match.isEmpty()) {
                checks.add(new Check(REQUIRE_EVENT_MARKERS, "marker '" + expected.triggerName() + "'",
                        "missing - add an event marker named '" + expected.triggerName()
                                + "' at waypoint position " + expected.position()));
            } else if (Math.abs(match.get().position() - expected.position()) > MARKER_POS_TOL) {
                checks.add(new Check(REQUIRE_EVENT_MARKERS, "marker '" + expected.triggerName() + "'",
                        "at position " + match.get().position() + ", expected ~" + expected.position()));
            } else {
                checks.add(new Check(true, "marker '" + expected.triggerName() + "'",
                        "present at position " + match.get().position()));
            }
        }
    }

    private static boolean verifyAutoSequence() {
        List<String> expected = List.of(
                "path:StartToPreloadF", "named:Score",
                "path:ReefFToStation", "named:Collect",
                "path:ReefDToStation", "named:Score");
        File autoFile = new File(Filesystem.getDeployDirectory(), "pathplanner/autos/SW205.auto");

        List<Check> checks = new ArrayList<>();
        List<String> actual = new ArrayList<>();
        if (!autoFile.isFile()) {
            checks.add(new Check(false, "file", "pathplanner/autos/SW205.auto not found"));
        } else {
            try (FileReader reader = new FileReader(autoFile)) {
                JSONObject root = (JSONObject) new JSONParser().parse(reader);
                collectAutoSteps((JSONObject) root.get("command"), actual);
            } catch (Exception e) {
                checks.add(new Check(false, "file", "could not parse SW205.auto: " + e.getMessage()));
            }
        }
        checks.add(new Check(actual.equals(expected), "command sequence",
                actual.equals(expected)
                        ? "matches: " + prettySequence(actual)
                        : "expected " + prettySequence(expected) + " but found "
                                + (actual.isEmpty() ? "nothing usable" : prettySequence(actual))));
        return report("SW205.auto", checks);
    }

    private static void collectAutoSteps(JSONObject command, List<String> out) {
        if (command == null) {
            return;
        }
        String type = (String) command.get("type");
        JSONObject data = (JSONObject) command.get("data");
        if ("sequential".equals(type) || "parallel".equals(type) || "race".equals(type)
                || "deadline".equals(type)) {
            for (Object o : (JSONArray) data.get("commands")) {
                collectAutoSteps((JSONObject) o, out);
            }
        } else if ("path".equals(type)) {
            out.add("path:" + data.get("pathName"));
        } else if ("named".equals(type)) {
            out.add("named:" + data.get("name"));
        }
    }

    private static String prettySequence(List<String> steps) {
        StringBuilder sb = new StringBuilder();
        for (String step : steps) {
            sb.append(sb.isEmpty() ? "" : " -> ").append(step.substring(step.indexOf(':') + 1));
        }
        return sb.toString();
    }

    private static boolean report(String name, List<Check> checks) {
        System.out.println("[Path] " + name);
        boolean pass = true;
        for (Check c : checks) {
            pass &= c.pass();
            System.out.printf(Locale.US, "   %s %-22s %s%n", c.pass() ? "[ OK ]" : "[FAIL]",
                    c.label(), c.detail());
        }
        System.out.println(pass ? "   => PASS" : "   => FAIL");
        return pass;
    }

    static Optional<PathPlannerTrajectory> generateIdeal(PathPlannerPath path, RobotConfig config) {
        try {
            Optional<PathPlannerTrajectory> ideal = path.getIdealTrajectory(config);
            if (ideal.isPresent()) {
                return ideal;
            }
            return Optional.of(path.generateTrajectory(new ChassisSpeeds(), Rotation2d.kZero, config));
        } catch (Exception e) {
            System.out.println("   [ERROR] Trajectory generation failed: " + e);
            return Optional.empty();
        }
    }

    static PoseError poseError(Pose2d expected, Pose2d actual) {
        return new PoseError(
                expected.getTranslation().getDistance(actual.getTranslation()),
                Math.abs(MathUtil.inputModulus(
                        expected.getRotation().minus(actual.getRotation()).getDegrees(),
                        -180.0, 180.0)));
    }

    static RouteError routeError(PathPlannerTrajectory reference, PathPlannerTrajectory student) {
        double maxPos = 0.0;
        double maxRot = 0.0;
        double worstTime = 0.0;
        double totalTime = reference.getTotalTimeSeconds();
        for (int i = 0; i <= ROUTE_SAMPLES; i++) {
            double t = totalTime * i / ROUTE_SAMPLES;
            var refState = reference.sample(t);
            var stuState = student.sample(Math.min(t, student.getTotalTimeSeconds()));
            double posErr =
                    refState.pose.getTranslation().getDistance(stuState.pose.getTranslation());
            double rotErr = Math.abs(MathUtil.inputModulus(
                    refState.pose.getRotation().minus(stuState.pose.getRotation()).getDegrees(),
                    -180.0, 180.0));
            if (posErr > maxPos) {
                maxPos = posErr;
                worstTime = t;
            }
            maxRot = Math.max(maxRot, rotErr);
        }
        return new RouteError(maxPos, maxRot, worstTime);
    }

    static boolean durationWithinTolerance(double referenceTime, double studentTime) {
        return Math.abs(studentTime - referenceTime)
                <= Math.max(TIME_ABS_TOL_S, TIME_REL_TOL * referenceTime);
    }

    private static String describePose(PoseError err) {
        return String.format(Locale.US, "dPos=%.3f m, dRot=%.1f deg", err.posErrM(), err.rotErrDeg());
    }

    private static void printBanner(String message) {
        System.out.println("====================================================================");
        System.out.println(" " + message);
        System.out.println("====================================================================");
    }

    private static Translation2d t(double x, double y) {
        return new Translation2d(x, y);
    }

    static PathConstraints lessonConstraints() {
        return new PathConstraints(5.7, 3.4, Math.toRadians(540), Math.toRadians(720), 12.3);
    }

    static PathPlannerPath referenceStartToPreloadF() {
        return new PathPlannerPath(
                List.of(
                        new Waypoint(null, t(7.0, 2.0), t(6.212153797590234, 2.138918542133544)),
                        new Waypoint(t(5.971345131623847, 2.0807466682572264), t(5.2, 3.0), null)),
                List.of(),
                List.of(),
                List.of(),
                List.of(new EventMarker("L4", 0.2)),
                lessonConstraints(),
                new IdealStartingState(0.25, Rotation2d.fromDegrees(180)),
                new GoalEndState(0.0, Rotation2d.fromDegrees(120)),
                false);
    }

    static PathPlannerPath referenceReefFToStation() {
        return new PathPlannerPath(
                List.of(
                        new Waypoint(null, t(5.2, 3.0), t(5.287155742747658, 2.0038053019082547)),
                        new Waypoint(t(2.63135401666597, 1.7607128711332591), t(1.0, 1.0), null)),
                List.of(),
                List.of(),
                List.of(),
                List.of(new EventMarker("IntakePosition", 0.5)),
                lessonConstraints(),
                new IdealStartingState(0.25, Rotation2d.fromDegrees(120)),
                new GoalEndState(0.0, Rotation2d.fromDegrees(55)),
                false);
    }

    static PathPlannerPath referenceReefDToStation() {
        return new PathPlannerPath(
                List.of(
                        new Waypoint(null, t(1.0, 1.0), t(3.165063509461097, 2.25)),
                        new Waypoint(t(3.180847955711008, 2.2264235636489538), t(4.0, 2.8), null)),
                List.of(),
                List.of(),
                List.of(),
                List.of(new EventMarker("L4", 0.5)),
                lessonConstraints(),
                new IdealStartingState(0.0, Rotation2d.fromDegrees(55)),
                new GoalEndState(0.0, Rotation2d.fromDegrees(60)),
                false);
    }
}
