package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants.SHOOTER_PARAMETERS;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class runShooterCommand extends Command{
    private final Shooter   shooter;
    private final Hood      hood;
    private final Transfer  transfer;
    private final Hopper    hopper;
    private final boolean   isPassing;
    private final double    distance;

    public runShooterCommand(Shooter    shooter,
                            Hood        hood,
                            Transfer    transfer,
                            Hopper      hopper,
                            boolean     isPassing,
                            double      distance){
        this.shooter    = shooter;
        this.hood       = hood;
        this.transfer   = transfer;
        this.hopper     = hopper;
        this.isPassing  = isPassing;
        this.distance   = distance;

        addRequirements(shooter, hood, transfer, hopper);
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
            transfer.transfer();
            hopper.run();
        } else {
            transfer.transferPercent(0);
            hopper.run(0);
        }
    }

    public void end(boolean interupted) {
        shooter.setRPM(0);
        transfer.transferPercent(0);
        hopper.run(0);
  }
}
