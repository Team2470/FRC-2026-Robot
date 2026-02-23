package frc.robot.autos.modes;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.autos.common.AutoBase;
// import frc.robot.commands.ShootWhileMoving;

public class LeftDepotClimb extends AutoBase{
    private static final Path startPath         = PathsBase.LEFT_START_TO_DEPOT;
    private static final Path depotToLeftClimb  = PathsBase.DEPOT_TO_LEFT_CLIMB;

    public LeftDepotClimb() {
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
        // Second step of Auto is to go from Depot to Climb
        // Shoot Depot fuel while moving
        // Run intake incase any other balls get in intaking range
        addCommands(new ParallelCommandGroup(
            followPathCommand(depotToLeftClimb.getPathPlannerPath())
            // ShootWhileMoving Auto Command
            // Intake Auto Command
            // Raise Climber
        ));

        // addCommands(Climb);
    }
}
