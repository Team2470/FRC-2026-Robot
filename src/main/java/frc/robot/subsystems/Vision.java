package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
public class Vision extends SubsystemBase{
    private static final String KEY_SEEN_TAG_IDS = "Vision/SeenTagIds";
    private static final String KEY_SEEN_TAG_HEADINGS_DEG = "Vision/SeenTagHeadingsDeg";
    private static final String KEY_SEEN_TAG_TURRET_ERROR_DEG = "Vision/SeenTagTurretErrorDeg";
    private final AprilTagFieldLayout fieldLayout;
    private Pose2d lastRobotPose = new Pose2d();
    private LimelightHelpers.RawFiducial[] lastRawFiducials = new LimelightHelpers.RawFiducial[0];
    private boolean hasRobotPose;
    double m_lastLimelightPrintTime;
    public Vision() {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.kDefaultField.m_resourceFile
            );
        } catch (IOException e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
    }
    public void findPose1(){
        double now = Timer.getFPGATimestamp();
        if (now - m_lastLimelightPrintTime < (1.0 / 12.0)) {
            return;
        }
        m_lastLimelightPrintTime = now;

        String limelightName = "limelight-intake";
        LimelightHelpers.PoseEstimate prePoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (prePoseEstimate == null) {
            hasRobotPose = false;
            lastRawFiducials = new LimelightHelpers.RawFiducial[0];
            System.out.println("Limelight pose estimate unavailable (" + limelightName + ").");
            return;
        } 
        Pose2d poseEstimate = prePoseEstimate.pose;
        var pose = poseEstimate;

        lastRobotPose = pose;
        lastRawFiducials =
            prePoseEstimate.rawFiducials == null ? new LimelightHelpers.RawFiducial[0] : prePoseEstimate.rawFiducials;
        hasRobotPose = true;
    }

    public void updateSeenTagsDashboard() {
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_IDS, getSeenTagIds());
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_HEADINGS_DEG, getSeenTagHeadingsDeg());
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_TURRET_ERROR_DEG, getSeenTagTurretErrorDeg());
    }

    private double[] getSeenTagIds() {
        if (lastRawFiducials == null || lastRawFiducials.length == 0) {
            return new double[0];
        }
        ArrayList<Double> ids = new ArrayList<>();
        for (LimelightHelpers.RawFiducial fiducial : lastRawFiducials) {
            if (fiducial == null) {
                continue;
            }
            double id = fiducial.id;
            if (!ids.contains(id)) {
                ids.add(id);
            }
        }
        double[] out = new double[ids.size()];
        for (int i = 0; i < ids.size(); i++) {
            out[i] = ids.get(i);
        }
        return out;
    }

    private double[] getSeenTagHeadingsDeg() {
        ArrayList<Double> ids = getSeenTagIdsList();
        if (ids.isEmpty()) {
            return new double[0];
        }
        double[] out = new double[ids.size()];
        for (int i = 0; i < ids.size(); i++) {
            Optional<Pose3d> tagPose3dOpt = fieldLayout.getTagPose(ids.get(i).intValue());
            if (tagPose3dOpt.isEmpty()) {
                out[i] = Double.NaN;
                continue;
            }
            Pose2d tagPose2d = tagPose3dOpt.get().toPose2d();
            Translation2d robotToTag = tagPose2d.getTranslation().minus(lastRobotPose.getTranslation());
            Rotation2d directionToTag = robotToTag.getAngle();
            out[i] = directionToTag.getDegrees();
        }
        return out;
    }

    private double[] getSeenTagTurretErrorDeg() {
        ArrayList<Double> ids = getSeenTagIdsList();
        if (ids.isEmpty()) {
            return new double[0];
        }
        double[] out = new double[ids.size()];
        for (int i = 0; i < ids.size(); i++) {
            Optional<Pose3d> tagPose3dOpt = fieldLayout.getTagPose(ids.get(i).intValue());
            if (tagPose3dOpt.isEmpty()) {
                out[i] = Double.NaN;
                continue;
            }
            Pose2d tagPose2d = tagPose3dOpt.get().toPose2d();
            Translation2d robotToTag = tagPose2d.getTranslation().minus(lastRobotPose.getTranslation());
            Rotation2d directionToTag = robotToTag.getAngle();
            double errorDeg = directionToTag.minus(lastRobotPose.getRotation()).getDegrees();
            out[i] = Math.IEEEremainder(errorDeg, 360.0);
        }
        return out;
    }

    private ArrayList<Double> getSeenTagIdsList() {
        ArrayList<Double> ids = new ArrayList<>();
        if (lastRawFiducials == null || lastRawFiducials.length == 0) {
            return ids;
        }
        for (LimelightHelpers.RawFiducial fiducial : lastRawFiducials) {
            if (fiducial == null) {
                continue;
            }
            double id = fiducial.id;
            if (!ids.contains(id)) {
                ids.add(id);
            }
        }
        return ids;
    }

    @Override public void periodic(){
    findPose1();
    updateSeenTagsDashboard();
    }

}
