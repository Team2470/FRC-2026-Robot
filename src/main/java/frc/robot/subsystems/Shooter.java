package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Constants.shooterConstants.SHOOTER_PARAMETERS;
import frc.robot.subsystems.Hood;
// import frc.robot.Constants.shooterConstants.Targets;


public class Shooter extends SubsystemBase {

    private final TalonFX m_topMotor_1 = new TalonFX(shooterConstants.FLYWHEEL_1_DEVICE_ID);
    private final TalonFX m_topMotor_2 = new TalonFX(shooterConstants.FLYWHEEL_2_DEVICE_ID);
    // private final TalonFX m_transferMotor                     = new TalonFX(shooterConstants.FEEDER_DEVICE_ID);
    // private final TalonFX m_hoodMotor                       = new TalonFX(shooterConstants.HOOD_DEVICE_ID);
    private final VelocityVoltage m_velocityRequest         = new VelocityVoltage(0);

    // private final TalonFX m_motor;
    // //placeholder constants for PID
    private double m_demand;
    // public shooterConstants.Targets targetNumber = shooterConstants.Targets.HUB;
    public double targetRPM = 500;
    public double targetAngle = 25;
    public double distance = 1.219;
    public double angle = 1.3;
    // private finasl PIDController m_pidController = new PIDController(shooterConstants.FLYWHEEL_KP, shooterConstants.FLYWHEEL_KI, shooterConstants.FLYWHEEL_KD);
    public Hood hood = new Hood();

    private enum ControlMode {
        kOpenLoop, kPID
    }

    private ControlMode m_controlMode = ControlMode.kOpenLoop;


    // formula for ballistic trajectory WITHOUT DRAG, change in the future to account for this if needed
    private Double angleCalculator(Double v,  Double x, Double y) {
        return(Math.atan((Math.pow(v, 2) + Math.sqrt(Math.pow(v, 4) - 9.8 * (9.8 * Math.pow(x,2) + 2 * y * Math.pow(v,2))))/ (9.8 * x)));
    }

    public Shooter() {

         TalonFXConfiguration config = new TalonFXConfiguration();
        // PID gains must be tuned for RPS (Phoenix 6 standard)
        config.Slot0.kP = shooterConstants.FLYWHEEL_KP;
        config.Slot0.kI = shooterConstants.FLYWHEEL_KI;
        config.Slot0.kD = shooterConstants.FLYWHEEL_KD;
        config.Slot0.kV = shooterConstants.FLYWHEEL_KV;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_topMotor_1.getConfigurator().apply(config);
        m_topMotor_2.getConfigurator().apply(config);
        m_topMotor_2.optimizeBusUtilization();
        m_topMotor_2.setControl(new Follower(m_topMotor_1.getDeviceID(), shooterConstants.FLYWHEEL_ALIGNMENT_VALUE));
    }

    public void periodic(){
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("TargetRPM", targetRPM);
        // SmartDashboard.putString("targetNumber", targetNumber.toString());
    }

    public SHOOTER_PARAMETERS getHUBParameters(double distance) {
        return shooterConstants.HUB_MAP.get(distance);
    }

    public SHOOTER_PARAMETERS getPassParameters(double distance) {
        return shooterConstants.PASS_MAP.get(distance);
    }
    public double getHubRPM(double distance) {
        return shooterConstants.HUB_MAP.get(distance).rpm();
    }

    public double getPassRPM(double distance) {
        return shooterConstants.PASS_MAP.get(distance).rpm();
    }

    public double getHubTOF(double distance) {
        return shooterConstants.HUB_MAP.get(distance).timeOfFlight();
    }

    public double getPassTOF(double distance) {
        return shooterConstants.PASS_MAP.get(distance).timeOfFlight();
    }

    public double getHoodHub(double distance) {
        return shooterConstants.HUB_MAP.get(distance).hoodAngle();
    }

    public double getHoodPass(double distance) {
        return shooterConstants.PASS_MAP.get(distance).hoodAngle();
    }

    public void setRPM(double rpm) {
        // Phoenix sends values in Rotations Per Seconds (RPS)
        // Must handle value accordingly
        m_topMotor_1.setControl(m_velocityRequest.withVelocity(rpm / Constants.MINUTE_TO_SECONDS));
        SmartDashboard.putNumber("Flywheel Velocity ", rpm);
    }

    public boolean isAtSpeed(double targetRPM, double tolerance) {
        // Get actual speed (Phoenix gives RPS), convert to RPM
        double currentRPM = m_topMotor_1.getVelocity().getValueAsDouble() * Constants.MINUTE_TO_SECONDS;

        // Tolerance: Is Current RPM = Target RPM +/- Tolerance (measured in RPM)
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    // public Command runShooterCommand(){
    //     return Commands.runEnd(
    //         () -> {
    //             this.setRPM(targetRPM);
    //         },
    //         () -> { this.setRPM(0);}, this);
    // }

    public Command increaseRPM(){
        return Commands.runOnce(
            () -> {
                this.targetRPM += 500;
            }, this);
    }

    public Command decreaseRPM(){
        return Commands.runOnce(
            () -> {
                this.targetRPM -= 500;
            }, this);
    }


    public void increaseDistance(){
        double newDistance  = this.distance + 0.05;
        // this.targetRPM      = getHubRPM(newDistance);
        // this.targetAngle    = getHoodHub(newDistance);
        // hood.setAngle(this.targetAngle);

        this.distance       = newDistance;
    }

    public void decreaseDistance(){
        double newDistance  = this.distance - 0.05;
        // this.targetRPM      = getHubRPM(newDistance);
        // this.targetAngle    = getHoodHub(newDistance);
        // hood.setAngle(this.targetAngle);
        this.distance       = newDistance;
    }

}