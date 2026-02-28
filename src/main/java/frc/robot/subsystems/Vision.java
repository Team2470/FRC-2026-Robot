package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
public class Vision extends SubsystemBase{
    double m_lastLimelightPrintTime;
    double poseX;
    double poseY; 
    double Rotation;
    public void findPose1(){
        double now = Timer.getFPGATimestamp();
        if (now - m_lastLimelightPrintTime < (1.0 / 12.0)) {
            return;
        }
        m_lastLimelightPrintTime = now;

        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        String limelightName = "limelight-shooter";
        var poseEstimate = isRed
                ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (poseEstimate == null) {
            System.out.println("Limelight pose estimate unavailable (" + limelightName + ").");
            return;
        }
        var pose = poseEstimate.pose;

        var poseX = pose.getX();
        var poseY = pose.getY();
        var rotation = pose.getRotation().getDegrees();

        SmartDashboard.putNumber("poseX", poseX);
        SmartDashboard.putNumber("poseY", poseY);
        SmartDashboard.putNumber("rotation", rotation);
        // System.out.printf(
        //         "Limelight pose estimate: x=%.2f y=%.2f rot=%.1f deg%n",
        //         poseX,
        //         poseY,
        //         rotation);
    }
    @Override public void periodic(){
    findPose1();
    }

}
