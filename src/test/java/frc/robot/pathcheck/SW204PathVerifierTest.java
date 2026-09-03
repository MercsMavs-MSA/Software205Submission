package frc.robot.pathcheck;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;

class SW205PathVerifierTest {
    private static final PathConstraints verySlowConstraints = new PathConstraints(1.0, 1.0,
            Math.toRadians(540), Math.toRadians(720), 12.3);

    @Test
    void deployedPathsMatchAnswerKey() {
        assertTrue(SW205PathVerifier.verifyAllAndPrint());
    }

    @Test
    void detectsMovedEndpoint() throws Exception {
        var refTraj = ideal(SW205PathVerifier.referenceReefFToStation());
        var badEnd = new PathPlannerPath(
                List.of(
                        new Waypoint(null, new Translation2d(5.2, 3.0),
                                new Translation2d(5.287155742747658, 2.0038053019082547)),
                        new Waypoint(new Translation2d(2.63135401666597, 1.7607128711332591),
                                new Translation2d(1.0, 0.55), null)),
                List.of(), List.of(), List.of(), List.of(),
                SW205PathVerifier.lessonConstraints(),
                new IdealStartingState(0.25, Rotation2d.fromDegrees(120)),
                new GoalEndState(0.0, Rotation2d.fromDegrees(55)),
                false);
        var err = SW205PathVerifier.poseError(
                refTraj.getEndState().pose, ideal(badEnd).getEndState().pose);
        assertTrue(err.posErrM() > SW205PathVerifier.START_END_POS_TOL_M,
                "moved endpoint should exceed tolerance, got " + err.posErrM());

        var route = SW205PathVerifier.routeError(refTraj, ideal(badEnd));
        assertTrue(route.maxPosErrM() > SW205PathVerifier.ROUTE_POS_TOL_M,
                "route deviation should flag moved endpoint, got " + route.maxPosErrM());
    }

    @Test
    void detectsTooSlowPath() throws Exception {
        var refTime = ideal(SW205PathVerifier.referenceReefFToStation()).getTotalTimeSeconds();
        var slow = new PathPlannerPath(
                List.of(
                        new Waypoint(null, new Translation2d(5.2, 3.0),
                                new Translation2d(5.287155742747658, 2.0038053019082547)),
                        new Waypoint(new Translation2d(2.63135401666597, 1.7607128711332591),
                                new Translation2d(1.0, 1.0), null)),
                List.of(), List.of(), List.of(), List.of(),
                verySlowConstraints,
                new IdealStartingState(0.25, Rotation2d.fromDegrees(120)),
                new GoalEndState(0.0, Rotation2d.fromDegrees(55)),
                false);
        double slowTime = ideal(slow).getTotalTimeSeconds();
        assertFalse(SW205PathVerifier.durationWithinTolerance(refTime, slowTime),
                "slow path should fail timing check: " + refTime + " vs " + slowTime);
    }

    private static PathPlannerTrajectory ideal(PathPlannerPath path) throws Exception {
        Optional<PathPlannerTrajectory> traj =
                SW205PathVerifier.generateIdeal(path, RobotConfig.fromGUISettings());
        assertTrue(traj.isPresent(), "trajectory generation failed");
        return traj.get();
    }
}
