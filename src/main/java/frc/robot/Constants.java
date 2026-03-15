package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }

    public static final double MINUTE_TO_SECONDS = 60.0;

    public static final Translation2d RED_HUB_LOCATION = new Translation2d(12.000, 4.025);
    public static final Translation2d BLUE_HUB_LOCATION = new Translation2d(4.600, 4.025);
    public static final Translation2d HUB_LOCATION = isBlueAlliance() ? BLUE_HUB_LOCATION : RED_HUB_LOCATION;

    public static final Translation2d RED_LEFT_PASS_LOCATION = new Translation2d(15.000, 6.000);
    public static final Translation2d RED_RIGHT_PASS_LOCATION = new Translation2d(15.000, 2.000);
    public static final Translation2d BLUE_LEFT_PASS_LOCATION = new Translation2d(2.000, 2.000);
    public static final Translation2d BLUE_RIGHT_PASS_LOCATION = new Translation2d(2.000, 6.000);
    public static final Translation2d LEFT_PASS_LOCATION = isBlueAlliance() ? BLUE_LEFT_PASS_LOCATION : RED_LEFT_PASS_LOCATION;
    public static final Translation2d RIGHT_PASS_LOCATION = isBlueAlliance() ? BLUE_RIGHT_PASS_LOCATION : RED_RIGHT_PASS_LOCATION;

    public static class QuestNavConstants{
        public static final Transform2d ROBOT_TO_QUEST = new Transform2d(
        new Translation2d(Inches.of((29.0 / 2) - 16.725), Inches.of((29.0 / 2.0) - 5.762)),
        Rotation2d.fromDegrees(180));
        //   public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
        // new Translation3d(Inches.of((29.0 / 2) - 16.725), Inches.of((29.0 / 2.0) - 5.762), Inches.of(0.0)),
        // new Rotation3d(Rotation2d.fromDegrees(180)));
        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.03, // X: Trust Quest to within 3cm (Trust more than odometry)
          0.03, // Y: Trust Quest to within 3cm
          0.5 // Theta: Trust Quest rotation LESS than Gyro (Trust Pigeon more)
        );
    }
    public static class shooterConstants {
        public static final int FLYWHEEL_1_DEVICE_ID = 1;
        public static final int FLYWHEEL_2_DEVICE_ID = 2;
        public static final double FLYWHEEL_KP = .2;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KD = 0;
        public static final double FLYWHEEL_KV = .125;
        public static final MotorAlignmentValue FLYWHEEL_ALIGNMENT_VALUE = MotorAlignmentValue.Opposed;

        // Turret Constants
        public static final int TURRET_DEVICE_ID                        = 0;
        public static final double TURRET_GEAR_RATIO                    = 50.0;
        public static final double MIN_TURRET_ANGLE                     = 0.0;
        public static final double MAX_TURRET_ANGLE                     = 180.0;
        public static final double MIN_HOOD_ANGLE                       = 25.0;
        public static final double MAX_HOOD_ANGLE                       = 45.0;
        public static final double MIN_TURRET_SOFT_LIMIT                = MIN_TURRET_ANGLE /
                                                                            360.0 * TURRET_GEAR_RATIO;
        public static final double MAX_TURRET_SOFT_LIMIT                = MAX_TURRET_ANGLE /
                                                                            360.0 * TURRET_GEAR_RATIO;
        public static final double TURRET_KP                            = 0.15;
        public static final double TURRET_KI                            = 0.0;
        public static final double TURRET_KD                            = 0.0;
        public static final double TURRET_KV                            = 0.105;
        public static final double TURRET_MOTION_MAGIC_CRUISE_VELOCITY  = 80.0;
        public static final double TURRET_MOTION_MAGIC_ACCELERACTIION   = 160.0;

        public static final Translation2d ROBOT_TO_TURRET = new Translation2d(0.187325, 0.16764);

        public static final double MIN_HUB_DISTANCE = 0.8128;
        public static final double MAX_HUB_DISTANCE = 6.477;

        public static final double MAX_PASS_DISTANCE = 11.000;
        public static final double MIN_PASS_DISTANCE = 1.500;

        public static InterpolatingDoubleTreeMap HUB_RPM_MAP    = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap PASS_RPM_MAP   = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap HOOD_HUB_MAP   = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap HOOD_PASS_MAP  = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap PASS_TOF_MAP   = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap HUB_TOF_MAP    = new InterpolatingDoubleTreeMap();

         static {
            // TODO: Actually test for these values.
            // Initial values are based on using desmos Trajectory Calculator
            // And do not reflect real-world-values

            // This map is for the Hood angle
            // when we are shooting into hub
            // Distance (meters), Hood Angle (degrees)
            HOOD_HUB_MAP.put(0.8128, 25.000);
            HOOD_HUB_MAP.put(2.997, 40.000);
            HOOD_HUB_MAP.put(6.477, 40.000);

            // This map is for the Hood angle
            // when we are passing into alliance zone
            // Distance (meters), Hood Angle (degrees)
            HOOD_PASS_MAP.put(1.524, 45.0);
            HOOD_PASS_MAP.put(3.048, 45.0);
            HOOD_PASS_MAP.put(6.096, 45.0);
            HOOD_PASS_MAP.put(7.620, 45.0);
            HOOD_PASS_MAP.put(9.144, 45.0);
            HOOD_PASS_MAP.put(11.280, 45.0);

            // This map is for the shooter flywheel
            // when we are shooting into hub
            // Distance (meters), Flywheel Speed (RPM)
            HUB_RPM_MAP.put(0.8128, 1700.000);
            HUB_RPM_MAP.put(2.997, 2100.000);
            HUB_RPM_MAP.put(6.477, 2600.000);

            // This map is for the shooter flywheel
            // when we are passing into alliance zone
            // Distance (meters), Flywheel Speed (RPM)
            PASS_RPM_MAP.put(1.524, 3539.700);
            PASS_RPM_MAP.put(3.048, 4601.610);
            PASS_RPM_MAP.put(6.096, 6902.415);
            PASS_RPM_MAP.put(7.620, 7079.400);
            PASS_RPM_MAP.put(9.144, 7100.000);
            PASS_RPM_MAP.put(11.280,7256.385);

            // This map is time of flight in seconds
            // when we are passing into alliance zone
            // Distance (meters), Time of Flight (Seconds)
            PASS_TOF_MAP.put(1.524, 1.000);
            PASS_TOF_MAP.put(3.048, 1.200);
            PASS_TOF_MAP.put(6.096, 1.400);
            PASS_TOF_MAP.put(7.620, 1.600);
            PASS_TOF_MAP.put(9.144, 1.900);
            PASS_TOF_MAP.put(11.280, 2.400);

            // This map is time of flight in seconds
            // when we are shooting into hub
            // Distance (meters), Time of Flight (Seconds)
            HUB_TOF_MAP.put(3.993, 1.900);
            HUB_TOF_MAP.put(3.048, 1.600);
            HUB_TOF_MAP.put(2.438, 1.200);
            HUB_TOF_MAP.put(1.829, 1.000);
            HUB_TOF_MAP.put(1.219, 0.900);
        }
    }
    public static class IntakePivotConstants {
        public static final boolean encoderDirection = false;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
    }
}