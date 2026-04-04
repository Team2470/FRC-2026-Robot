
package frc.robot.subsystems.Localization;
import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;
import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.geometry.Translation3d;
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

        String[] limelights = {"limelight-shooter", "limelight-intake"};
        PoseEstimate poseEstimates = null;

        for(int i = 0; i < limelights.length - 1; i++){
            poseEstimates = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelights[i]);
        
            if(isValidPoseEstimate(poseEstimates)){
                double adjustedXYDeviation = 0.05 + (0.01 * Math.pow(poseEstimates.avgTagDist, 2));
                SmartDashboard.putNumber("LL Std Devs", adjustedXYDeviation);
                Matrix<N3, N1> adjustedDeviations = VecBuilder.fill(adjustedXYDeviation, adjustedXYDeviation, Double.MAX_VALUE);
                visionMeasurementConsumer.addVisionMeasurement(poseEstimates.pose.toPose2d(), poseEstimates.timestampSeconds, adjustedDeviations);
                
                if(bestDeviation > adjustedXYDeviation){
                    bestEstimate = poseEstimates;
                    bestDeviation = adjustedXYDeviation;
                }

                bestLimelightPose = bestEstimate == null ? null: bestEstimate.pose;
                return bestEstimate;
            }
        }
        return bestEstimate;
    }

   public void findQuestPose(){
        questNav.commandPeriodic();

        PoseFrame[] newFrames = questNav.getAllUnreadPoseFrames();

        SmartDashboard.putBoolean("questNav/isConnected", questNav.isConnected());
        SmartDashboard.putBoolean("questNav/isTracking", questNav.isTracking());

        for (int i = newFrames.length - 1; i >= 0; i--) {
            if (questNav.isConnected() && questNav.isTracking()) {
                questRobotPose = newFrames[i].questPose3d().transformBy(new Transform3d(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse()));

                SmartDashboard.putNumber("questNav/Frame x", newFrames[i].questPose3d().getX());
                SmartDashboard.putNumber("questNav/Frame y", newFrames[i].questPose3d().getY());
                SmartDashboard.putNumber("questNav/Frame Rotation", newFrames[i].questPose3d().getRotation().getAngle());

                visionMeasurementConsumer.addVisionMeasurement(questRobotPose.toPose2d(), newFrames[i].dataTimestamp(), QUESTNAV_STD_DEVS);
                break; // Found the most recent tracking frame, exit loop
            }
        }
    }

    public void setQuestNavPose(Pose3d robotPose) {
        Pose3d questPose = robotPose.transformBy(new Transform3d(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse()));
        if(questPose.getX() < 0.0 || questPose.getX() > 16.500 || questPose.getY() < 0.0 || questPose.getY() > 8.00){
            return;
        }
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
        SmartDashboard.putNumber("robotAngle",robotAngle.getRotations());
        if (inAllianceZone) {
            // Robot Angle is the angle of the robot on the field
            // angle to Hub is the angle of the center of the robot to the hub
            angleToHub = Constants.HUB_LOCATION.minus(poseSupplier.get().getTranslation()).getAngle();
            turretAngle = angleToHub.minus(robotAngle);
            SmartDashboard.putNumber("turretHubAngle", turretAngle.getRotations());
            SmartDashboard.putNumber("angleToHub", angleToHub.getRotations());
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
        if (DriverStation.isDisabled()) {
            ResetPoseCommand();
        }
        PoseEstimate bestEstimate = findLLPose();
        if(bestEstimate != null){
            SmartDashboard.putNumber("LL Best Estimate X", bestEstimate.pose.getX());
            SmartDashboard.putNumber("LL Best Estimate Y", bestEstimate.pose.getY());
            SmartDashboard.putNumber("LL Best Estimate Rotation", bestEstimate.pose.getRotation().getAngle());
        }
        findQuestPose();
        checkAllianceZone();
    //  checkVisibility();
        // if(inAllianceZone){
            setHubDistance();
        // } else {
            setPassDistance();
        Rotation2d angle = getTurretAngle();
        turret.setTargetAngle(angle.times(-1));
        SmartDashboard.putNumber("Hub X", Constants.HUB_LOCATION.getX());
        SmartDashboard.putNumber("Hub Y", Constants.HUB_LOCATION.getY());
    }

    public void ResetPoseCommand() {
        // if (DriverStation.isDisabled()){
        //     if(Constants.isBlueAlliance()) {
        //         // Blue Alliance Pathplanner Start Position
        //         setQuestNavPose(new Pose2d(new Translation2d(4.450, 7.350), Rotation2d.kZero));
        //     } else {
        //         // Red Alliance Pathplanner Start Position
        //         setQuestNavPose(new Pose2d(new Translation2d(12.100, 0.750), Rotation2d.k180deg));
        //     }
        // } else{
            LimelightHelpers.PoseEstimate resetPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
            if(resetPose.tagCount < 2.0) {
                LimelightHelpers.PoseEstimate intakePose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-intake");
                if(intakePose.tagCount < 2.0) {
                    return;
                } else {
                    setQuestNavPose(intakePose.pose);
                    visionMeasurementConsumer.addVisionMeasurement(intakePose.pose.toPose2d(), intakePose.timestampSeconds, QUESTNAV_STD_DEVS);
                }
            } else {
                setQuestNavPose(resetPose.pose);
                visionMeasurementConsumer.addVisionMeasurement(resetPose.pose.toPose2d(), resetPose.timestampSeconds, QUESTNAV_STD_DEVS);
            }
        // }
    }

    public Pose3d getLimelightRotationPose() {
        Pose3d rotatedLimelightPose = LimelightHelpers.getCameraPose3d_RobotSpace("limelight-shooter");
        Rotation3d turretRotation3d = new Rotation3d(turret.getTurretAngle());
        Translation3d turretCenterTranslation3d = new Translation3d(shooterConstants.ROBOT_TO_TURRET);
        return rotatedLimelightPose.rotateAround(turretCenterTranslation3d, turretRotation3d);
    }

}
