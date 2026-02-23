package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.MathHelpers;

public class RobotState {
    private CommandSwerveDrivetrain drive;
    private Pose2d autoStartPose;
    private static RobotState INSTANCE;

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }

        return INSTANCE;
    }

    private RobotState() {}

    // public Pose2d getFieldToRobot() {
    //     if (drive.getPose() != null) {
    //         return drive.getPose();
    //     }

    //     return MathHelpers.POSE_2D_ZERO;
    // }

    // public ChassisSpeeds getChassisSpeeds() {
    //     return drive.getFieldRelativeSpeeds();
    // }

    public void setAutoStartPose(Pose2d startPose) {
        this.autoStartPose = startPose;
    }

    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red) ? true : false;
        } else {
            return false;
        }
    }

}
