package frc.robot.autos.modes;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.autos.common.AutoBase;
// import frc.robot.commands.ShootWhileMoving;

public class LeftNZRightClimb extends AutoBase{
    private static final Path startPath             = PathsBase.LEFT_START_TO_DEPOT;
    private static final Path depotToLeftBumpAZ     = PathsBase.DEPOT_TO_LEFT_BUMP_AZ;
    private static final Path leftBumpAZToNZ        = PathsBase.LEFT_BUMP_AZ_TO_NZ;
    private static final Path leftBumpNZToNZStart   = PathsBase.LEFT_BUMP_NZ_TO_NZ_START;
    private static final Path leftToRightNZ         = PathsBase.LEFT_TO_RIGHT_NZ;
    private static final Path rightNZToRightTrench  = PathsBase.RIGHT_NZ_TO_RIGHT_TRENCH;
    private static final Path rightTrenchLoop       = PathsBase.RIGHT_TRENCH_LOOP;
    private static final Path rightTrenchToNZ       = PathsBase.RIGHT_TRENCH_TO_NZ;
    private static final Path rightToLeftNZ         = PathsBase.RIGHT_TO_LEFT_NZ;

    public LeftNZRightClimb() {
        // Set Starting Pose
        super(startPath.getPathPlannerPath().getStartingHolonomicPose());
    }

    public void init() {
        // Immediately in Auto, drive towards depot
        // While shooting Pre-load
        // Start running intake to get depot fuel
        addCommands(new ParallelCommandGroup(
            followPathCommand(startPath.getPathPlannerPath())
            // ShootWhileMoving Auto Command
            // Run Hopper/Feeder
            // Intake Auto Command
        ));
        // Second step of Auto is to go from Depot to Bump Aligment
        // Shoot Depot fuel while moving to Bump
        // Run intake incase any other balls get in intaking range
        addCommands(new ParallelCommandGroup(
            followPathCommand(depotToLeftBumpAZ.getPathPlannerPath())
            // ShootWhileMoving Auto Command
            // Run Hopper/Feeder
            // Intake Auto Command
        ));
        // When traversing bump, only
        addCommands(
            followPathCommand(leftBumpAZToNZ.getPathPlannerPath())
        );
        // Get into sweeping position
        // Run intake and auto pass for any loose fuel
        addCommands(new ParallelCommandGroup(
            followPathCommand(leftBumpNZToNZStart.getPathPlannerPath())
            // PassWhileMoving Auto Command
            // Run Hopper/Feeder
            // Intake Auto Command
        ));
        // Sweep
        // Run intake for full path
        // PassWhileMoving for most, but not entire path
        // Give time to fill hopper for shooting volley later
        addCommands(new ParallelCommandGroup(
            followPathCommand(leftToRightNZ.getPathPlannerPath())
            // PassWhileMoving Auto Command with 2 second race/deadline
            // Run Hopper/Feeder with 2 second race/deadline
            // Intake Auto Command
        ));
        // Get to the trench to get ready to shoot
        addCommands(new ParallelCommandGroup(
            followPathCommand(rightNZToRightTrench.getPathPlannerPath())
            // Only run intake, grab any loose fuel
        ));

        // Once pass the trench, start shooting
        addCommands(new ParallelCommandGroup(
            followPathCommand(rightTrenchLoop.getPathPlannerPath())
            // ShootWhileMoving Auto Command
            // Run Hopper/Feeder
            // Intake Auto Command
        ));
        // Get ready to sweep again
        addCommands(new ParallelCommandGroup(
            followPathCommand(rightTrenchToNZ.getPathPlannerPath())
            // Intake Auto Command
        ));
        // Sweep
        // Run intake for full path
        // PassWhileMoving for most, but not entire path
        // Give time to fill hopper for shooting volley later
        addCommands(new ParallelCommandGroup(
            followPathCommand(rightToLeftNZ.getPathPlannerPath())
            // PassWhileMoving Auto Command with 2 second race/deadline
            // Run Hopper/Feeder with 2 second race/deadline
            // Intake Auto Command
        ));

    }
}
