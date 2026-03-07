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
    private static final String KEY_SEEN_TAG_DISTANCES_M = "Vision/SeenTagDistancesM";
    private static final String KEY_SEEN_TAG_CAMERA_ERROR_DEG = "Vision/SeenTagCameraErrorDeg";
    private static final String KEY_CLOSEST_TAG_ID = "Vision/ClosestSeenTagId";
    private static final String KEY_CLOSEST_TAG_DISTANCE_M = "Vision/ClosestTagDistanceM";
    public static final String KEY_CLOSEST_TAG_CAMERA_ERROR_DEG = "Vision/ClosestTagCameraErrorDeg";
    private static final String LIMELIGHT_NAME = "limelight-intake";
    private final AprilTagFieldLayout fieldLayout;
    private Pose2d lastRobotPose = new Pose2d();
    private LimelightHelpers.RawFiducial[] lastRawFiducials = new LimelightHelpers.RawFiducial[0];
    private boolean hasRobotPose;
    double m_lastLimelightPrintTime;
    

    public static final class SeenTagInfo {
        private final int id;
        private final double distanceM;
        private final double cameraErrorDeg;

        private SeenTagInfo(int id, double distanceM, double cameraErrorDeg) {
            this.id = id;
            this.distanceM = distanceM;
            this.cameraErrorDeg = cameraErrorDeg;
        }

        public int getId() {
            return id;
        }

        public double getDistanceM() {
            return distanceM;
        }

        public double getCameraErrorDeg() {
            return cameraErrorDeg;
        }
    }
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

        LimelightHelpers.PoseEstimate prePoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if (prePoseEstimate == null) {
            hasRobotPose = false;
            lastRawFiducials = new LimelightHelpers.RawFiducial[0];
            System.out.println("Limelight pose estimate unavailable (" + LIMELIGHT_NAME + ").");
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
        SeenTagInfo[] tags = getSeenTagInfo();
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_IDS, toIdArray(tags));
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_DISTANCES_M, toDistanceArray(tags));
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_CAMERA_ERROR_DEG, toCameraErrorArray(tags));

        SeenTagInfo closest = getClosestTagInfo();
        SmartDashboard.putNumber(KEY_CLOSEST_TAG_ID, closest == null ? Double.NaN : closest.id);
        SmartDashboard.putNumber(KEY_CLOSEST_TAG_DISTANCE_M, closest == null ? Double.NaN : closest.distanceM);
        SmartDashboard.putNumber(KEY_CLOSEST_TAG_CAMERA_ERROR_DEG, closest == null ? Double.NaN : closest.cameraErrorDeg);
    }

    /**
     * Builds a compact per-tag view in a single pass over raw fiducials.
     */
    private SeenTagInfo[] getSeenTagInfo() {
        if (lastRawFiducials == null || lastRawFiducials.length == 0) {
            return new SeenTagInfo[0];
        }
        ArrayList<SeenTagInfo> out = new ArrayList<>();
        for (LimelightHelpers.RawFiducial fid : lastRawFiducials) {
            if (fid == null) {
                continue;
            }
            boolean exists = false;
            for (SeenTagInfo info : out) {
                if (info.id == fid.id) {
                    exists = true;
                    break;
                }
            }
            if (exists) {
                continue;
            }
            out.add(new SeenTagInfo(fid.id, fid.distToCamera, fid.txnc));
        }
        return out.toArray(new SeenTagInfo[0]);
    }

    private double[] toIdArray(SeenTagInfo[] tags) {
        double[] out = new double[tags.length];
        for (int i = 0; i < tags.length; i++) {
            out[i] = tags[i].id;
        }
        return out;
    }

    private double[] toDistanceArray(SeenTagInfo[] tags) {
        double[] out = new double[tags.length];
        for (int i = 0; i < tags.length; i++) {
            out[i] = tags[i].distanceM;
        }
        return out;
    }

    private double[] toCameraErrorArray(SeenTagInfo[] tags) {
        double[] out = new double[tags.length];
        for (int i = 0; i < tags.length; i++) {
            out[i] = tags[i].cameraErrorDeg;
        }
        return out;
    }

    public SeenTagInfo getClosestTagInfo() {
        SeenTagInfo[] tags = getSeenTagInfo();
        if (tags.length == 0) {
            return null;
        }
        SeenTagInfo closest = tags[0];
        for (int i = 1; i < tags.length; i++) {
            if (tags[i].distanceM < closest.distanceM) {
                closest = tags[i];
            }
        }
        return closest;
    }

    /**
     * Returns how many degrees the camera/turret should rotate to center the tag in view.
     * Uses Limelight "tx" (horizontal offset) where 0 means centered.
     */
    public double getTurretAimErrorDeg() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }

    @Override public void periodic(){
        findPose1();
        updateSeenTagsDashboard();
    }

}
