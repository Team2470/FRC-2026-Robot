package frc.robot;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.shooterConstants.SHOOTER_PARAMETERS;

public class Constants {
    public static final double MINUTE_TO_SECONDS = 60.0;
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isEmpty()
                || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }


    public static final class fieldConstants {
        // Target Locations
        public static final Translation2d BLUE_HUB_LOCATION             = new Translation2d(4.600, 4.025);
        public static final Translation2d RED_HUB_LOCATION              = new Translation2d(12.000, 4.025);
        public static final Translation2d BLUE_RIGHT_PASS_LOCATION      = new Translation2d(2.500, 2.000);
        public static final Translation2d BLUE_LEFT_PASS_LOCATION       = new Translation2d(2.500, 6.000);
        public static final Translation2d RED_LEFT_PASS_LOCATION        = new Translation2d(14.500, 2.000);
        public static final Translation2d RED_RIGHT_PASS_LOCATION       = new Translation2d(14.500, 6.000);
        public static final Translation2d HUB_LOCATION                  = isBlueAlliance() ? BLUE_HUB_LOCATION :
                                                                                                RED_HUB_LOCATION;
        public static final Translation2d PASS_LEFT_LOCATION            = isBlueAlliance() ? BLUE_LEFT_PASS_LOCATION :
                                                                                                RED_LEFT_PASS_LOCATION;
        public static final Translation2d PASS_RIGHT_LOCATION           = isBlueAlliance() ? BLUE_RIGHT_PASS_LOCATION :
                                                                                                RED_RIGHT_PASS_LOCATION;
    }

    public static class transferConstants {
        public static final int TRANSFER_DEVICE_ID = 7;
        public static final double RUN_TRANSFER_VOLTAGE     = 12;
        public static final double REVERSE_TRANSFER_VOLTAGE = -12;

    }

    public static class hopperConstants {
        public static final int HOPPER_DEVICE_ID = 9;
        public static final double RUN_HOPPER_VOLTAGE     = 12;
        public static final double REVERSE_HOPPER_VOLTAGE = -12;
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

         // TODO: grab coordinates of Center of Turret compared to our robot's origin point (typically in the center of our bellypan)
        public static final Transform3d ROBOT_TO_TURRET = new Transform3d(-1.0, 0.0, 0.44, Rotation3d.kZero);

        public static final double MAX_HUB_DISTANCE = 4.000;
        public static final double MIN_HUB_DISTANCE = 1.200;

        public static final double MAX_PASS_DISTANCE = 13.000;
        public static final double MIN_PASS_DISTANCE = 1.500;

        public record SHOOTER_PARAMETERS(double rpm, double hoodAngle, double timeOfFlight){}

        public static final Interpolator<SHOOTER_PARAMETERS> SHOOTER_PARAM_INTERPOLATOR =
            (start, end, t) -> {
                double interpRPM = start.rpm() + (end.rpm() - start.rpm()) * t;
                double interpHood = start.hoodAngle() + (end.hoodAngle() - start.hoodAngle()) * t;
                double interpTime = start.timeOfFlight() + (end.timeOfFlight() - start.timeOfFlight()) * t;
                return new SHOOTER_PARAMETERS(interpRPM, interpHood, interpTime);
            };

        public static InterpolatingTreeMap<Double, SHOOTER_PARAMETERS> HUB_MAP  = new InterpolatingTreeMap
                                                                                        <Double, SHOOTER_PARAMETERS>
                                                                                        (InverseInterpolator.forDouble(),
                                                                                        SHOOTER_PARAM_INTERPOLATOR);
        public static InterpolatingTreeMap<Double, SHOOTER_PARAMETERS> PASS_MAP = new InterpolatingTreeMap
                                                                                        <Double, SHOOTER_PARAMETERS>
                                                                                        (InverseInterpolator.forDouble(),
                                                                                        SHOOTER_PARAM_INTERPOLATOR);

        static {
            // TODO: Get good values for passing
            PASS_MAP.put(1.000, new SHOOTER_PARAMETERS(1500.000, 0.000, 0.800));
            PASS_MAP.put(2.000, new SHOOTER_PARAMETERS(2000.000, 0.000, 0.900));
            // PASS_MAP.put( 1.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 2.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 3.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 4.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 5.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 6.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 7.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 8.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 9.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put(10.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put(11.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put(12.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put(13.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));

            // TODO: Get good values for the HUB
            HUB_MAP.put(1.000, new SHOOTER_PARAMETERS(1800.000, 25.000, 1.100));
            HUB_MAP.put(4.000, new SHOOTER_PARAMETERS(2500.000, 25.000, 1.900));
            // 0.500 Meters ( 1.640 feet)
            // HUB_MAP.put(0.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 1.000 Meters ( 3.281 feet)
            // HUB_MAP.put(1.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 1.500 Meters ( 4.921 feet)
            // HUB_MAP.put(1.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 2.000 Meters ( 6.562 feet)
            // HUB_MAP.put(2.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 2.500 Meters ( 8.202 feet)
            // HUB_MAP.put(2.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 3.000 Meters ( 9.843 feet)
            // HUB_MAP.put(3.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 3.500 Meters (11.483 feet)
            // HUB_MAP.put(3.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 4.000 Meters (13.123 feet)
            // HUB_MAP.put(4.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 4.500 Meters (14.764 feet)
            // HUB_MAP.put(4.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 5.000 Meters (16.404 feet)
            // HUB_MAP.put(5.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 5.500 Meters (18.045 feet)
            // HUB_MAP.put(5.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 6.000 Meters (19.685 feet)
            // Just past real max value of shooting inside alliance zone
            // HUB_MAP.put(6.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
        }
    }
}