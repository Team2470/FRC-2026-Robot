package frc.robot;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    public static final double MINUTE_TO_SECONDS = 60.0;
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
        public static final double MIN_HOOD_ANGLE                     = 25.0;
        public static final double MAX_HOOD_ANGLE                     = 65.0;
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
            HOOD_HUB_MAP.put(3.993, 25.000); 
            // HOOD_HUB_MAP.put(3.048, 67.000);
            // HOOD_HUB_MAP.put(2.438, 65.000);
            // HOOD_HUB_MAP.put(1.829, .000);
            HOOD_HUB_MAP.put(1.219, 45.000);
        
            // This map is for the Hood angle 
            // when we are passing into alliance zone
            // Distance (meters), Hood Angle (degrees)
            HOOD_PASS_MAP.put(1.524, 40.0); 
            HOOD_PASS_MAP.put(3.048, 30.0);
            HOOD_PASS_MAP.put(6.096, 18.0);
            HOOD_PASS_MAP.put(7.620, 23.0);
            HOOD_PASS_MAP.put(9.144, 30.0);
            HOOD_PASS_MAP.put(11.280, 35.0);
        
            // This map is for the shooter flywheel
            // when we are shooting into hub
            // Distance (meters), Flywheel Speed (RPM)
            HUB_RPM_MAP.put(3.993, 4247.640);
            HUB_RPM_MAP.put(3.048, 4070.655);
            HUB_RPM_MAP.put(2.438, 3716.685);
            HUB_RPM_MAP.put(1.829, 3539.700);
            HUB_RPM_MAP.put(1.219, 3716.685);
        
            // This map is for the shooter flywheel
            // when we are passing into alliance zone
            // Distance (meters), Flywheel Speed (RPM)
            PASS_RPM_MAP.put(1.524, 3539.700);
            PASS_RPM_MAP.put(3.048, 4601.610);
            PASS_RPM_MAP.put(6.096, 6902.415);
            PASS_RPM_MAP.put(7.620, 7079.400);
            PASS_RPM_MAP.put(9.144, 6725.430);
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
}
