package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
import java.io.IOException;
import java.util.Optional;
public class Vision extends SubsystemBase{
    private static final String KEY_TARGET_TAG_ID = "Vision/TargetTagId";
    private static final String KEY_FACING_THRESHOLD_DEG = "Vision/FacingThresholdDeg";
    private static final String KEY_ROBOT_HEADING_DEG = "Vision/RobotHeadingDeg";
    private static final String KEY_HEADING_TO_TAG_DEG = "Vision/HeadingToTagDeg";
    private static final String KEY_HEADING_ERROR_DEG = "Vision/HeadingErrorDeg";
    private static final String KEY_IS_FACING_TAG = "Vision/IsFacingTag";
    private static final String KEY_HAS_ROBOT_POSE = "Vision/HasRobotPose";
    private static final String KEY_HAS_TAG_POSE = "Vision/HasTagPose";
    private final AprilTagFieldLayout fieldLayout;
    private Pose2d lastRobotPose = new Pose2d();
    private boolean hasRobotPose;
    double m_lastLimelightPrintTime;
    double poseX;
    double poseY; 
    double Rotation;
    double distanceToHub;
    public Vision() {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.kDefaultField.m_resourceFile
            );
        } catch (IOException e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
        SmartDashboard.setDefaultNumber(KEY_TARGET_TAG_ID, 1);
        SmartDashboard.setDefaultNumber(KEY_FACING_THRESHOLD_DEG, 20.0);
    }
    public void findPose1(){
        double now = Timer.getFPGATimestamp();
        if (now - m_lastLimelightPrintTime < (1.0 / 12.0)) {
            return;
        }
        m_lastLimelightPrintTime = now;

        var alliance = DriverStation.getAlliance();
        //boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        String limelightName = "limelight-intake";
        LimelightHelpers.PoseEstimate prePoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (prePoseEstimate == null) {
            hasRobotPose = false;
            System.out.println("Limelight pose estimate unavailable (" + limelightName + ").");
            return;
        } 
        Pose2d poseEstimate = prePoseEstimate.pose;
        var pose = poseEstimate;

        poseX = pose.getX();
        poseY = pose.getY();
        Rotation = pose.getRotation().getDegrees();
        lastRobotPose = pose;
        hasRobotPose = true;

        SmartDashboard.putNumber("poseX", poseX);
        SmartDashboard.putNumber("poseY", poseY);
        SmartDashboard.putNumber("rotation", Rotation);
        // System.out.printf(
        //         "Limelight pose estimate: x=%.2f y=%.2f rot=%.1f deg%n",
        //         poseX,
        //         poseY,
        //         rotation);
    }

    public void setDistance() {
        distanceToHub = Math.sqrt((11.915394- poseX) *(11.915394- poseX) + (4.03 - poseY) *(4.03 - poseY));
        SmartDashboard.putNumber("distanceToHub", distanceToHub);
    }

    public void updateFacingTagDashboard() {
        SmartDashboard.putBoolean(KEY_HAS_ROBOT_POSE, hasRobotPose);
        if (!hasRobotPose) {
            SmartDashboard.putNumber(KEY_ROBOT_HEADING_DEG, Double.NaN);
            SmartDashboard.putNumber(KEY_HEADING_TO_TAG_DEG, Double.NaN);
            SmartDashboard.putNumber(KEY_HEADING_ERROR_DEG, Double.NaN);
            SmartDashboard.putBoolean(KEY_HAS_TAG_POSE, false);
            SmartDashboard.putBoolean(KEY_IS_FACING_TAG, false);
            return;
        }

        double targetTagIdRaw = SmartDashboard.getNumber(KEY_TARGET_TAG_ID, 0);
        int targetTagId = (int) Math.round(targetTagIdRaw);
        double maxAngleErrorDeg = SmartDashboard.getNumber(KEY_FACING_THRESHOLD_DEG, 20.0);

        SmartDashboard.putNumber(KEY_ROBOT_HEADING_DEG, lastRobotPose.getRotation().getDegrees());

        Optional<Pose3d> tagPose3dOpt = fieldLayout.getTagPose(targetTagId);
        SmartDashboard.putBoolean(KEY_HAS_TAG_POSE, tagPose3dOpt.isPresent());
        if (tagPose3dOpt.isEmpty()) {
            SmartDashboard.putNumber(KEY_HEADING_TO_TAG_DEG, Double.NaN);
            SmartDashboard.putNumber(KEY_HEADING_ERROR_DEG, Double.NaN);
            SmartDashboard.putBoolean(KEY_IS_FACING_TAG, false);
            return;
        }

        Pose2d tagPose2d = tagPose3dOpt.get().toPose2d();
        Translation2d robotToTag = tagPose2d.getTranslation().minus(lastRobotPose.getTranslation());
        Rotation2d directionToTag = robotToTag.getAngle();
        double errorDeg = directionToTag.minus(lastRobotPose.getRotation()).getDegrees();
        errorDeg = Math.IEEEremainder(errorDeg, 360.0);

        SmartDashboard.putNumber(KEY_HEADING_TO_TAG_DEG, directionToTag.getDegrees());
        SmartDashboard.putNumber(KEY_HEADING_ERROR_DEG, errorDeg);
        SmartDashboard.putBoolean(KEY_IS_FACING_TAG, Math.abs(errorDeg) <= maxAngleErrorDeg);
    }

    public boolean isFacingTag(Pose2d robotPose, int seenTagId, double maxAngleErrorDeg) {
        Optional<Pose3d> tagPose3dOpt = fieldLayout.getTagPose(seenTagId);
        if (tagPose3dOpt.isEmpty()) {
            return false;
        }

        Pose2d tagPose2d = tagPose3dOpt.get().toPose2d();
        Translation2d robotToTag = tagPose2d.getTranslation().minus(robotPose.getTranslation());
        Rotation2d directionToTag = robotToTag.getAngle();
        Rotation2d robotHeading = robotPose.getRotation();

        double errorDeg = directionToTag.minus(robotHeading).getDegrees();
        errorDeg = Math.IEEEremainder(errorDeg, 360.0);

        return Math.abs(errorDeg) <= maxAngleErrorDeg;
    }

    @Override public void periodic(){
    findPose1();
    setDistance();
    updateFacingTagDashboard();
    }

}
