package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants.SHOOTER_PARAMETERS;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class runShooterCommand extends Command{
    private final Shooter   shooter;
    private final Hood      hood;
    private final Feeder    feeder;
    private final Hopper    hopper;
    private final boolean   isPassing;
    private final double    distance;

    public runShooterCommand(Shooter    shooter,
                            Hood        hood,
                            Feeder      feeder,
                            Hopper      hopper,
                            boolean     isPassing,
                            double      distance){
        this.shooter    = shooter;
        this.hood       = hood;
        this.feeder     = feeder;
        this.hopper     = hopper;
        this.isPassing  = isPassing;
        this.distance   = distance;

        addRequirements(shooter, hood, feeder, hopper);
    }

    @Override
    public void execute(){
        SHOOTER_PARAMETERS shotParameters;
        if(isPassing){
            shotParameters = shooter.getPassParameters(distance);
        } else {
            shotParameters = shooter.getHUBParameters(distance);
        }

        shooter.setRPM(shotParameters.rpm());
        hood.setAngle(shotParameters.hoodPosition());

        if(shooter.isAtSpeed(shotParameters.rpm(), 50) &&
            hood.isPositionWithinTolerance()){
            feeder.run();
            hopper.run();
        }
    }
}
