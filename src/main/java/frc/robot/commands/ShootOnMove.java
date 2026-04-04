package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Localization.Vision;

public class ShootOnMove extends Command{
    private final Shooter   shooter;
    private final Hood      hood;
    private final Transfer  transfer;
    private final Hopper    hopper;
    private final Vision    vision;
    private final Turret    turret;
    private final boolean   isPassing;
    private final boolean   passLeft;
    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotVelocity;

    public ShootOnMove( Shooter                 shooter,
                        Hood                    hood,
                        Transfer                transfer,
                        Hopper                  hopper,
                        Vision                  vision,
                        Turret                  turret,
                        boolean                 isPassing,
                        Boolean                 passLeft,
                        Supplier<Pose2d>        robotPose,
                        Supplier<ChassisSpeeds> robotVelocity){

        this.shooter        = shooter;
        this.hood           = hood;
        this.transfer       = transfer;
        this.hopper         = hopper;
        this.vision         = vision;
        this.turret         = turret;
        this.isPassing      = isPassing;
        this.passLeft       = passLeft;
        this.robotPose      = robotPose;
        this.robotVelocity  = robotVelocity;
    addRequirements(shooter, hood, transfer, hopper, vision, turret);
}
    @Override
    public void execute(){
        Pose2d robot = robotPose.get();
        ChassisSpeeds robotSpeed = robotVelocity.get();
        Translation2d shooterTranslation = robot.getTranslation().plus(shooterConstants.ROBOT_TO_TURRET.rotateBy(robot.getRotation()));
        Translation2d target = null;

        if (isPassing) {
            if(passLeft){
                target = Constants.LEFT_PASS_LOCATION;
            } else {
                target = Constants.RIGHT_PASS_LOCATION;
            }
        } else {
            target = Constants.HUB_LOCATION;
        }
        
        double distToTarget = shooterTranslation.getDistance(target);
        Translation2d robotMovement = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        double omega = robotSpeed.omegaRadiansPerSecond;

        Translation2d robotToShooter = shooterTranslation.minus(robot.getTranslation());
        Translation2d vTan = new Translation2d(-omega * robotToShooter.getY(), omega * robotToShooter.getX());

        Translation2d effectiveShooterVelocity = robotMovement.plus(vTan);

        if(hood.isPositionWithinTolerance() && shooter.isAtSpeed(TargetRPM, 25)){
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