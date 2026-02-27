package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.fieldConstants.isValidFieldTranslation;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.fieldConstants;
import frc.robot.Constants.visionConstants;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;
// import gg.questnav.questnav.PoseFrame;

public class VisionSubsystem extends SubsystemBase {
    private final Consumer<Pose2d> poseResetConsumer;
    private final VisionMeasurementConsumer visionMeasurementConsumer;
    private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    private Pose2d startingPose = new Pose2d();
    private final String cameraName = visionConstants.LIMELIGHT_SHOOTER;
    public VisionSubsystem(
        VisionMeasurementConsumer addVisionMeasurement,
        Consumer<Pose2d> poseResetConsumer,
        Supplier<Pose2d> poseSupplier) {

    this.visionMeasurementConsumer      = addVisionMeasurement;
    this.poseResetConsumer              = poseResetConsumer;

    LimelightHelpers.setCameraPose_RobotSpace("limelight-shooter",
            visionConstants.ROBOT_TO_LIMELIGHT_TRANSFORM3D.getX(),
            visionConstants.ROBOT_TO_LIMELIGHT_TRANSFORM3D.getY(),
            visionConstants.ROBOT_TO_LIMELIGHT_TRANSFORM3D.getZ(),
            visionConstants.ROBOT_TO_LIMELIGHT_TRANSFORM3D.getRotation().getMeasureX().in(Degrees),
            visionConstants.ROBOT_TO_LIMELIGHT_TRANSFORM3D.getRotation().getMeasureY().in(Degrees),
            visionConstants.ROBOT_TO_LIMELIGHT_TRANSFORM3D.getRotation().getMeasureZ().in(Degrees));
    }

    /**
     * Sets the starting pose for the robot at the beginning of a match.
     *
     * @param pose starting pose ALWAYS on the blue side, this will be flipped if on the red alliance
     */
    public void setInitialPose(Pose2d pose) {
      startingPose = pose;
    }

    /**
     * Resets the robot's pose to the given new pose.
     *
     * @param newPose the new pose to reset to (this is absolute and will not be flipped based on the alliance)
     */
    public void resetPose(Pose3d newPose) {
      poseResetConsumer.accept(newPose.toPose2d());
    }

    @Override
    public void periodic(){
        PoseEstimate bestEstimate = null;
        double shotDistance = 0.0;
        if(DriverStation.isDisabled()){
            periodicDisabled();
        } else {
            bestEstimate = periodicEnabled();
        }
        shotDistance = distanceToTarget(bestEstimate);
        SmartDashboard.putNumber("shotDistance", shotDistance);
        SmartDashboard.putNumberArray("bestEstimate", new double []{
            bestEstimate.pose.getX(),
            bestEstimate.pose.getY(),
            bestEstimate.pose.getRotation().getDegrees()});
    }

    /**
     * Validates a pose estimate based on common criteria.
     * <p>
     * Checks if the pose has valid tag count, is within field boundaries, and is within the tag distance threshold.
     *
     * @param poseEstimate the pose estimate to validate
     * @return true if the pose estimate meets basic validation criteria
     */
    private boolean isValidPoseEstimate(PoseEstimate poseEstimate) {
    //   return poseEstimate != null && poseEstimate.tagCount > 0
        //   && isValidFieldTranslation(poseEstimate.pose.getTranslation())
        //   && poseEstimate.avgTagDist < visionConstants.TAG_DISTANCE_THRESHOLD.in(Meters);
      return poseEstimate != null && poseEstimate.tagCount > 0;
    }

    /**
     * Handles periodic updates when the robot is disabled.
     * <p>
     * Validates AprilTag poses from the camera, and resets the pose estimator and QuestNav
     */
    private void periodicDisabled() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isBlueAlliance = alliance.isEmpty() || alliance.get() == Alliance.Blue;
        // Set the pipeline based on alliance color
        if (alliance.isEmpty() || alliance.get() == Alliance.Blue) {
            LimelightHelpers.setPipelineIndex(cameraName, visionConstants.LIMELIGHT_BLUE_PIPELINE);
        } else {
            LimelightHelpers.setPipelineIndex(cameraName, visionConstants.LIMELIGHT_RED_PIPELINE);
        }
        if (RobotState.isAutonomous() || DriverStation.isFMSAttached()) {
            // Preparing to run auto, set the starting pose to the start pose
            Pose2d allianceStartingPose = isBlueAlliance ? startingPose : FlippingUtil.flipFieldPose(startingPose);
            poseResetConsumer.accept(allianceStartingPose);
        }
    }

    /**
     * Handles periodic updates when the robot is enabled.
     * <p>
     * Processes all AprilTag cameras, validates poses, adds vision measurements with adjusted standard deviations,
     * and returns the best pose estimate for comparison with QuestNav.
     *
     * @return the best validated pose estimate from all cameras, or null if no valid estimates
     */
    private PoseEstimate periodicEnabled() {
        PoseEstimate bestEstimate = null;
        double bestDeviation = Double.MAX_VALUE;

        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        if (isValidPoseEstimate(poseEstimate)) {

            double baseStandardDeviations = visionConstants.APRILTAG_STD_DEVS;

            double adjustedXYDeviation = baseStandardDeviations + (0.01 * Math.pow(poseEstimate.avgTagDist, 2));
            Matrix<N3, N1> adjustedDeviations = VecBuilder.fill(adjustedXYDeviation, adjustedXYDeviation, Double.MAX_VALUE);
            visionMeasurementConsumer.addVisionMeasurement(
                                                            poseEstimate.pose,
                                                            poseEstimate.timestampSeconds,
                                                            adjustedDeviations);

            // Track the best estimate for QuestNav comparison
            // if (bestDeviation > adjustedXYDeviation) {
              bestEstimate = poseEstimate;
            //   bestDeviation = adjustedXYDeviation;
            // }
        }
      return bestEstimate;
    }

    private double distanceToTarget(PoseEstimate robotPose){
        return fieldConstants.HUB_LOCATION.getDistance(robotPose.pose.getTranslation());
    }
}