package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class ShooterCommand extends Command{
    private final Shooter   shooter;
    private final Hood      hood;
    private final Transfer  transfer;
    private final Hopper    hopper;    

    public ShooterCommand(Shooter    shooter,
                         Hood        hood,
                         Transfer    transfer,
                         Hopper      hopper){

        this.shooter    = shooter;
        this.hood       = hood;
        this.transfer   = transfer;
        this.hopper     = hopper;
    addRequirements(shooter, hood, transfer, hopper);
}
    @Override  
    public void execute(){
        double TargetRPM = shooter.getHubRPM(shooter.distance);
        double TargetHoodAngle = shooter.getHoodHub(shooter.distance);

        hood.setAngle(TargetHoodAngle);
        shooter.setRPM(TargetRPM);

        if(hood.isPositionWithinTolerance() && shooter.isAtSpeed(TargetRPM, 50)){
            transfer.transferPercent(12);
            hopper.hopperPercent(12);
        }
    }
    
}