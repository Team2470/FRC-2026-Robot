
package frc.robot.subsystems.Localization;
import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;
import static edu.wpi.first.math.util.Units.degreesToRadians;

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
import frc.robot.Constants;
import frc.robot.VisionMeasurementConsumer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Localization.LimelightHelpers.PoseEstimate;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import frc.robot.Constants.shooterConstants;

public class Vision extends SubsystemBase{
    public double poseX;
    public double poseY;
    public double Rotation;
    public double distanceToHub;
    public double distanceToPass;
    public boolean passLeft;
    public Rotation2d turretAngle;
    public boolean inAllianceZone = true;
    private Pose3d bestLimelightPose;
    private Pose3d questRobotPose;
    public Turret turret = new Turret();
    private Rotation2d robotAngle;
    private Rotation2d angleToHub;
    private Rotation2d angleToPassLeft;
    private Rotation2d angleToPassRight;

    QuestNav questNav = new QuestNav();
    double m_lastLimelightPrintTime;
    double targetX;
    private final VisionMeasurementConsumer visionMeasurementConsumer;
    private Supplier<Pose2d> poseSupplier;


    public Vision(VisionMeasurementConsumer addVisionMeasurement,
                    Supplier<Pose2d> poseSupplier){
        this.visionMeasurementConsumer = addVisionMeasurement;
        this.poseSupplier = poseSupplier;
        // LimelightHelpers.setCameraPose_RobotSpace("limelight-shooter", 0.199, 0.337, 0.429, 0.0, degreesToRadians(70), 0);
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

   public void findQuestPose(){
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
        passLeft = Constants.LEFT_PASS_LOCATION.getDistance(poseSupplier.get().getTranslation()) <
                            Constants.RIGHT_PASS_LOCATION.getDistance(poseSupplier.get().getTranslation());
        distanceToPass = Math.min(Constants.LEFT_PASS_LOCATION.getDistance(poseSupplier.get().getTranslation()),
                                    Constants.RIGHT_PASS_LOCATION.getDistance(poseSupplier.get().getTranslation()));
        distanceToPass = Math.max(shooterConstants.MIN_PASS_DISTANCE, Math.min(shooterConstants.MAX_PASS_DISTANCE, distanceToPass));
        SmartDashboard.putNumber("distanceToPass", distanceToPass);
        SmartDashboard.putBoolean("passLeft", passLeft);
    }

    public void setHubDistance() {
        distanceToHub = Constants.HUB_LOCATION.getDistance(poseSupplier.get().getTranslation());
        SmartDashboard.putNumber("preclampHubDist", distanceToHub);
        distanceToHub = Math.max(shooterConstants.MIN_HUB_DISTANCE, Math.min(shooterConstants.MAX_HUB_DISTANCE, distanceToHub));
        SmartDashboard.putNumber("distanceToHub", distanceToHub);
    }

    public void checkAllianceZone() {
        double x_value = poseSupplier.get().getX();
        if (Constants.isBlueAlliance() && (x_value > 5.500)) {
            inAllianceZone = false;
        } else if (!Constants.isBlueAlliance() && (x_value < 11.000)) {
            inAllianceZone = false;
        } else{
            inAllianceZone = true;
        }
        SmartDashboard.putBoolean("inAllianceZone" , inAllianceZone);
    }

    public Rotation2d getTurretAngle(){
        robotAngle = poseSupplier.get().getRotation();

        if (inAllianceZone) {
            // Robot Angle is the angle of the robot on the field
            // angle to Hub is the angle of the center of the robot to the hub
            angleToHub = Constants.HUB_LOCATION.minus(poseSupplier.get().getTranslation()).getAngle();
            turretAngle = angleToHub.minus(robotAngle);
            SmartDashboard.putNumber("turretHubAngle", turretAngle.getRotations());
        } else {
            if (passLeft) {
                angleToPassLeft = Constants.LEFT_PASS_LOCATION.minus(poseSupplier.get().getTranslation()).getAngle();
                turretAngle = angleToPassLeft.minus(robotAngle);
                SmartDashboard.putNumber("angleToPass", turretAngle.getRotations());
            } else {
                angleToPassRight = Constants.RIGHT_PASS_LOCATION.minus(poseSupplier.get().getTranslation()).getAngle();
                turretAngle = angleToPassRight.minus(robotAngle);
                SmartDashboard.putNumber("angleToPass", turretAngle.getRotations());
            }
        }
        return turretAngle.plus(Rotation2d.fromRotations(0.3));
        // return turretAngle; 
        
    }

    // public void checkVisibility() {
    //     if (LimelightHelpers.getTargetCount("limelight-shooter") > 0){
    //         targetX = LimelightHelpers.getTX("limelight-shooter");
    //         SmartDashboard.putNumber("targetX",targetX);
    //         Rotation2d rTargetX = turret.turretAngle.rotateBy( Rotation2d.fromDegrees(targetX));
    //         SmartDashboard.putNumber("rTargetX", rTargetX.getDegrees());
    //         turret.setTargetAngle(turret.turretAngle.rotateBy( Rotation2d.fromDegrees(targetX)));
    //     } else {
    //         return;
    //     }
    // }

    @Override public void periodic() {
        // PoseEstimate bestEstimate = findLLPose();
        findQuestPose();
        checkAllianceZone();
    //  checkVisibility();
        // if(inAllianceZone){
            setHubDistance();
        // } else {
            setPassDistance();
        // }
        // Rotation2d angle = getTurretAngle();
        // turret.setTargetAngle(angle.times(-1));

        if (DriverStation.isDisabled()) {
            ResetPoseCommand();
        }
    }

    public void ResetPoseCommand() {
        if (DriverStation.isDisabled()){
            if(Constants.isBlueAlliance()) {
                // Blue Alliance Pathplanner Start Position
                setQuestNavPose(new Pose2d(new Translation2d(4.450, 7.350), Rotation2d.kZero));
            } else {
                // Red Alliance Pathplanner Start Position
                setQuestNavPose(new Pose2d(new Translation2d(12.100, 0.750), Rotation2d.k180deg));
            }
        } else{
            LimelightHelpers.PoseEstimate resetPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
            if(resetPose.tagCount < 2.0) {
               return;
            } else {
               setQuestNavPose(resetPose.pose);
               visionMeasurementConsumer.addVisionMeasurement(resetPose.pose.toPose2d(), resetPose.timestampSeconds, QUESTNAV_STD_DEVS);
            }
        }
    }
}
