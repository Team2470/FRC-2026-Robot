package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Localization.Vision;

public class ShooterCommand extends Command{
    private final Shooter   shooter;
    private final Hood      hood;
    private final Transfer  transfer;
    private final Hopper    hopper;
    private final Vision    vision;
    private final boolean   isPassing;

    public ShooterCommand(Shooter    shooter,
                         Hood        hood,
                         Transfer    transfer,
                         Hopper      hopper,
                         Vision      vision,
                         boolean     isPassing){

        this.shooter    = shooter;
        this.hood       = hood;
        this.transfer   = transfer;
        this.hopper     = hopper;
        this.vision     = vision;
        this.isPassing  = isPassing;
    addRequirements(shooter, hood, transfer, hopper, vision);
}
    @Override
    public void execute(){
        double TargetRPM;
        double TargetHoodAngle;
        Rotation2d TargetTurretAngle;
        double distance;
        if(isPassing){
            distance = vision.distanceToPass;
            TargetRPM = shooter.getPassRPM(distance);
            TargetHoodAngle = shooter.getHoodPass(distance);
        } else {
            distance = vision.distanceToHub;
            TargetRPM = shooter.getHubRPM(distance);
            TargetHoodAngle = shooter.getHoodHub(distance);
            TargetTurretAngle = vision.turretHubAngle;
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
        hood.setAngle(25);
  }
}