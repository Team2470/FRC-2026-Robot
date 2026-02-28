package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class ShooterCommand extends Command{
    private final Shooter   shooter;
    private final Hood      hood;
    private final Transfer  transfer;
    private final Hopper    hopper;
    private final double    distance;
    private final boolean   isPassing;

    public ShooterCommand(Shooter    shooter,
                         Hood        hood,
                         Transfer    transfer,
                         Hopper      hopper,
                         double      distance,
                         boolean     isPassing){

        this.shooter    = shooter;
        this.hood       = hood;
        this.transfer   = transfer;
        this.hopper     = hopper;
        this.distance   = distance;
        this.isPassing  = isPassing;
    addRequirements(shooter, hood, transfer, hopper);
}
    @Override
    public void execute(){
        double TargetRPM;
        double TargetHoodAngle;
        double clampedDistance;
        if(isPassing){
            clampedDistance = Math.max(shooterConstants.MIN_PASS_DISTANCE,
                                Math.min(shooterConstants.MAX_PASS_DISTANCE, distance));
            TargetRPM = shooter.getPassRPM(clampedDistance);
            TargetHoodAngle = shooter.getHoodPass(clampedDistance);
        } else {
            clampedDistance = Math.max(shooterConstants.MIN_HUB_DISTANCE,
                                Math.min(shooterConstants.MAX_HUB_DISTANCE, shooter.distance));
            TargetRPM = shooter.getHubRPM(clampedDistance);
            TargetHoodAngle = shooter.getHoodHub(clampedDistance);
        }
        hood.setAngle(TargetHoodAngle);
        shooter.setRPM(TargetRPM);

        if(hood.isPositionWithinTolerance() && shooter.isAtSpeed(TargetRPM, 50)){
            transfer.transferPercent(12);
            hopper.hopperPercent(12);
        }
    }

    public void end(boolean interupted) {
        shooter.setRPM(0);
        transfer.transferPercent(0);
        hopper.hopperPercent(0);
  }
}