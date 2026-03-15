
package frc.robot.subsystems.Localization;

import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Localization.LimelightHelpers;
import frc.robot.subsystems.Localization.LimelightHelpers.PoseEstimate;
import gg.questnav.questnav.PoseFrame;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.Constants.QuestNavConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import frc.robot.Constants;
import frc.robot.VisionMeasurementConsumer;

public class Vision extends SubsystemBase{
    public double poseX;
    public double poseY;
    public double Rotation;
    public double distanceToHub;
    public double distanceToPass;
    double m_lastLimelightPrintTime;
    double targetX;
    String limelightName = "limelight-shooter";
    QuestNav questNav = new QuestNav();
    Turret turret = new Turret();
    private final VisionMeasurementConsumer visionMeasurementConsumer;
    private Supplier<Pose2d> poseSupplier;

    private Pose3d bestLimelightPose;
    private Pose3d questRobotPose;

    public Vision(VisionMeasurementConsumer addVisionMeasurement,
                    Supplier<Pose2d> poseSupplier){
        this.visionMeasurementConsumer = addVisionMeasurement;
        this.poseSupplier = poseSupplier;
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
        }
        bestLimelightPose = bestEstimate == null ? null: bestEstimate.pose;
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

    /**
    * Sets the QuestNav pose from the given 2D robot pose.
    * <p>
    * This sets the 3D pose with a Z of 0 (on the floor) and no pitch or roll.
    *
    * @param pose2d the robot's 2D pose
    */
    public void setQuestNavPose(Pose2d pose2d) {
      setQuestNavPose(new Pose3d(pose2d));
    }

    public void setHubDistance() {
        distanceToHub = Constants.HUB_LOCATION.getDistance(poseSupplier.get().getTranslation());
        SmartDashboard.putNumber("distanceToHub", distanceToHub);
    }

    public boolean setPassDistance(){
        boolean passLeft = Constants.LEFT_PASS_LOCATION.getDistance(poseSupplier.get().getTranslation()) <
                            Constants.RIGHT_PASS_LOCATION.getDistance(poseSupplier.get().getTranslation());
        distanceToPass = Math.min(Constants.LEFT_PASS_LOCATION.getDistance(poseSupplier.get().getTranslation()),
                                    Constants.RIGHT_PASS_LOCATION.getDistance(poseSupplier.get().getTranslation()));
        SmartDashboard.putNumber("distanceToPass", distanceToPass);
        SmartDashboard.putBoolean("passLeft", passLeft);
        return passLeft;
    }

    public void checkVisibility() {
        if (LimelightHelpers.getTargetCount("limelight-turret") > 0){
            targetX = LimelightHelpers.getTX("limelight-turret");
            SmartDashboard.putNumber("targetX",targetX);
            Rotation2d rTargetX = turret.turretAngle.rotateBy( Rotation2d.fromDegrees(targetX));
            SmartDashboard.putNumber("rTargetX", rTargetX.getDegrees());
            turret.setTargetAngle(turret.turretAngle.rotateBy( Rotation2d.fromDegrees(targetX)));
        } else {
            return;
        }
    }

    @Override public void periodic(){
        PoseEstimate bestEstimate = findLLPose();
        findQuestPose(bestEstimate);
        checkVisibility();
        setHubDistance();
        setPassDistance();
    }

    public void resetPoseCommand(){
        LimelightHelpers.PoseEstimate resetPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if(resetPose.tagCount < 2.0){
            return;
        } else {
            setQuestNavPose(resetPose.pose);
        }
    }

}
