
package frc.robot.subsystems.Localization;
import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;
import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.VisionMeasurementConsumer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Localization.LimelightHelpers.PoseEstimate;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Vision extends SubsystemBase{
    public double poseX;
    public double poseY; 
    public double Rotation;
    public double distanceToHub;
    public double distanceToPass;
    public boolean passLeft;
    public Rotation2d turretHubAngle;
    public Rotation2d angleToPass;
    private Pose3d bestLimelightPose;
    private Pose3d questRobotPose;

    QuestNav questNav = new QuestNav();
    Turret turret = new Turret();
    double m_lastLimelightPrintTime;
    double targetX;

    private final VisionMeasurementConsumer visionMeasurementConsumer;
    private Supplier<Pose2d> poseSupplier;


    public Vision(VisionMeasurementConsumer addVisionMeasurement,
                    Supplier<Pose2d> poseSupplier){
        this.visionMeasurementConsumer = addVisionMeasurement;
        this.poseSupplier = poseSupplier;
        LimelightHelpers.setCameraPose_RobotSpace("limelight-shooter", 0.199, 0.337, 0.429, 0.0, degreesToRadians(70), 0);
                                                    // When Turned to face hub pre-match: 0.222, 0.033, 0.435, 0.0, degreesToRadians(70), Math.PI/4
    }

    private boolean isValidPoseEstimate(PoseEstimate poseEstimate) {
        return poseEstimate != null && poseEstimate.tagCount > 1;
    }

    public PoseEstimate findLLPose(){
        PoseEstimate bestEstimate = null;
        double bestDeviation = Double.MAX_VALUE;

        String limelight = "limelight-shooter";

        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

        if(isValidPoseEstimate(poseEstimate)){
            double adjustedXYDeviation = 0.05 + (0.01 * Math.pow(poseEstimate.avgTagDist, 2));
            if(bestDeviation > adjustedXYDeviation){
                bestEstimate = poseEstimate;
                bestDeviation = adjustedXYDeviation;
            }

            bestLimelightPose = bestEstimate == null ? null: bestEstimate.pose;
            return bestEstimate;
        }
        return bestEstimate;
    }
       
    public void findQuestPose(PoseEstimate bestLLEstimate){
        questNav.commandPeriodic();

        PoseFrame[] newFrames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame frame : newFrames) {
            if (questNav.isConnected() && questNav.isTracking()) {
                questRobotPose = frame.questPose3d().transformBy(new Transform3d(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse()));

                SmartDashboard.putNumber("questNav/Frame x", frame.questPose3d().getX());
                SmartDashboard.putNumber("questNav/Frame y", frame.questPose3d().getY());
                SmartDashboard.putNumber("questNav/Frame z", frame.questPose3d().getZ());

                visionMeasurementConsumer.addVisionMeasurement(questRobotPose.toPose2d(), frame.dataTimestamp(), QUESTNAV_STD_DEVS);
                break; // Found the most recent tracking frame, exit loop
            }
        }
    }

    public void setQuestNavPose(Pose3d robotPose) {
        Pose3d questPose = robotPose.transformBy(new Transform3d(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse()));
        questNav.setPose(questPose);
    }

    public void setQuestNavPose(Pose2d pose2d) {
        questNav.setPose(new Pose3d(pose2d));
    }

    public void setPassDistance() {

    }

    public void setHubDistance() {
        distanceToHub = Constants.HUB_LOCATION.getDistance(poseSupplier.get().getTranslation());
        SmartDashboard.putNumber("distanceToHub", distanceToHub);
    }

    public void getHubAngle(){
        Rotation2d robotAngle = poseSupplier.get().getRotation();
        Rotation2d angleToHub = Constants.HUB_LOCATION.minus(poseSupplier.get().getTranslation()).getAngle();
        turretHubAngle = angleToHub.minus(robotAngle);
        SmartDashboard.putNumber("turretHubAngle", turretHubAngle.getRotations());
    }

    public void checkVisibility() {
        if (LimelightHelpers.getTargetCount("limelight-shooter") > 0){
            targetX = LimelightHelpers.getTX("limelight-shooter");
            SmartDashboard.putNumber("targetX",targetX);
            Rotation2d rTargetX = turret.turretAngle.rotateBy( Rotation2d.fromDegrees(targetX));
            SmartDashboard.putNumber("rTargetX", rTargetX.getDegrees());
            turret.setTargetAngle(turret.turretAngle.rotateBy( Rotation2d.fromDegrees(targetX)));
        } else {
            return;
        }
    }

    @Override public void periodic() {
        PoseEstimate bestEstimate = findLLPose();
        findQuestPose(bestEstimate);
        checkVisibility();
        setHubDistance();
        setPassDistance();
        getHubAngle();
    }

    public void ResetPoseCommand() {
         LimelightHelpers.PoseEstimate resetPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
         if(resetPose.tagCount < 2.0) {
            return;
         } else {
            setQuestNavPose(resetPose.pose);
         }

    }
}
