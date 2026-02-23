package frc.robot.autos.common;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public abstract class AutoBase extends SequentialCommandGroup {
    protected final AutoFactory autoFactory = AutoFactory.getInstance();
    private CommandSwerveDrivetrain drivetrain;
    private Pose2d startPose;


    protected AutoBase(Optional<Pose2d> pathStartPose) {
        if (pathStartPose.isEmpty()) {
            startPose = new Pose2d();
        } else {
            startPose = pathStartPose.get();
        }

        if (RobotState.getInstance().isRedAlliance()) {
            startPose = FlippingUtil.flipFieldPose(startPose);
        }

        setStartPose(startPose);
        RobotState.getInstance().setAutoStartPose(startPose);
    }

    public abstract void init(); // defined in each Auto class

    private void setStartPose(Pose2d pathStartPose) {
        addCommands(new InstantCommand(() -> drivetrain.resetPose(pathStartPose)));
    }

    protected Command manualZero() {
        return new InstantCommand(() -> drivetrain.seedFieldCentric());
    }

    protected Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    protected static PathPlannerPath getPathFromFile(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return path;
        } catch (Exception e) {
            DriverStation.reportError(
                    "FAILED TO GET PATH FROM PATHFILE" + pathName + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    public static Optional<Pose2d> getStartPoseFromAutoFile(String autoName) {
        try {
            List<PathPlannerPath> pathList = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            return (pathList.get(0).getStartingHolonomicPose());
        } catch (Exception e) {
            DriverStation.reportError(
                    "Couldn't get starting pose from auto file: " + autoName + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    public static class Path { // combines access to pathplanner and choreo
        private String pathPlannerPathName;

        public Path(String PPName) {
            pathPlannerPathName = PPName;
        }

        public PathPlannerPath getPathPlannerPath() {
            try {
                return getPathFromFile(pathPlannerPathName);

            } catch (Exception e) {
                DriverStation.reportError(
                        "FAILED TO GET PATH FROM PATHFILE " + pathPlannerPathName + e.getMessage(), e.getStackTrace());
                return null;
            }
        }
    }

    public static final class PathsBase {
        // Auto Start Paths
        public static final Path LEFT_START_TO_DEPOT            = new Path("LeftStartToDepot");
        public static final Path RIGHT_START_TO_OUTPOST         = new Path("RightStartToOutpost");

        // Left Side Paths
        public static final Path DEPOT_TO_LEFT_BUMP_AZ          = new Path("DepotToLeftBumpAZ");
        public static final Path DEPOT_TO_LEFT_CLIMB            = new Path("DepotToLeftClimb");
        public static final Path LEFT_BUMP_AZ_TO_NZ             = new Path("LeftBumpAZToNZ");
        public static final Path LEFT_BUMP_NZ_TO_AZ             = new Path("LeftBumpNZToAZ");
        public static final Path LEFT_BUMP_NZ_TO_NZ_START       = new Path("LeftBumpNZToNZStart");
        public static final Path LEFT_NZ_TO_BUMP_NZ             = new Path("LeftNZToBumpNZ");
        public static final Path LEFT_TO_RIGHT_NZ               = new Path("LeftToRightNZ");
        public static final Path LEFT_TRENCH_TO_NZ              = new Path("LeftTrenchToNZ");
        public static final Path LEFT_NZ_TO_TRENCH              = new Path("LeftNZToTrench");

        // Right Side Paths
        public static final Path OUTPOST_TO_RIGHT_TRENCH        = new Path("OutpostToRightTrech");
        public static final Path RIGHT_NZ_TO_RIGHT_TRENCH       = new Path("RightNZToRightTrench");
        public static final Path RIGHT_TO_LEFT_NZ               = new Path("RightToLeftNZ");
        public static final Path RIGHT_TRENCH_TO_CLIMB          = new Path("RightTrenchToClimb");
        public static final Path RIGHT_TRENCH_TO_NZ             = new Path("RightTrenchToNZ");
        public static final Path RIGHT_TRENCH_LOOP              = new Path("RightTrenchLoop");
        public static final Path RIGHT_TRENCH_TO_LEFT_BUMP_AZ   = new Path("RightTrenchToLeftBumpAZ");
    }
}
