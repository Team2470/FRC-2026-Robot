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
    private static final String LIMELIGHT_NAME = "limelight-intake";
    private final AprilTagFieldLayout fieldLayout;
    private Pose2d lastRobotPose = new Pose2d();
    private LimelightHelpers.RawFiducial[] lastRawFiducials = new LimelightHelpers.RawFiducial[0];
    private boolean hasRobotPose;
    double m_lastLimelightPrintTime;
    
    // turret
    private static final Turret m_turret = new Turret();
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
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_IDS, getSeenTagIds());
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_DISTANCES_M, getSeenTagDistancesM());
        SmartDashboard.putNumberArray(KEY_SEEN_TAG_CAMERA_ERROR_DEG, getSeenTagCameraErrorDeg());
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

    /**
     * Per-seen-tag camera-frame horizontal error in degrees (Limelight "tx").
     * 0 means centered in the camera image for that tag.
     */
    private double[] getSeenTagCameraErrorDeg() {
        ArrayList<Double> ids = getSeenTagIdsList();
        if (ids.isEmpty()) {
            return new double[0];
        }
        double[] out = new double[ids.size()];
        for (int i = 0; i < ids.size(); i++) {
            int id = ids.get(i).intValue();
            double txDeg = Double.NaN;
            for (LimelightHelpers.RawFiducial fid : lastRawFiducials) {
                if (fid != null && fid.id == id) {
                    txDeg = fid.txnc;
                    break;
                }
            }
            out[i] = txDeg;
        }
        return out;
    }

    /**
     * Returns per-seen-tag distance to the camera in meters.
     * Output order matches {@link #getSeenTagIdsList()}.
     */
    private double[] getSeenTagDistancesM() {
        ArrayList<Double> ids = getSeenTagIdsList();
        if (ids.isEmpty()) {
            return new double[0];
        }
        double[] out = new double[ids.size()];
        for (int i = 0; i < ids.size(); i++) {
            int id = ids.get(i).intValue();
            double distM = Double.NaN;
            for (LimelightHelpers.RawFiducial fid : lastRawFiducials) {
                if (fid != null && fid.id == id) {
                    distM = fid.distToCamera;
                    break;
                }
            }
            out[i] = distM;
        }
        return out;
    }

    /**
     * Returns the camera-frame horizontal error (tx, degrees) for the closest seen tag.
     * If no tags are seen, returns NaN.
     */
    public double getClosestTagCameraErrorDeg() {
        if (lastRawFiducials == null || lastRawFiducials.length == 0) {
            return Double.NaN;
        }
        LimelightHelpers.RawFiducial closest = null;
        for (LimelightHelpers.RawFiducial fid : lastRawFiducials) {
            if (fid == null) {
                continue;
            }
            if (closest == null || fid.distToCamera < closest.distToCamera) {
                closest = fid;
            }
        }
        if (closest == null) {
            return Double.NaN;
        }
        return closest.txnc;
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
