package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.IntakePivotConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakePivot extends SubsystemBase {
    //change id
    private final TalonFX m_motor;
    private final CANcoder m_encoder;
    private double m_demand;
    private ControlMode m_controlMode = ControlMode.kOpenLoop;
    private double uplimit = 50;
    private double upPosition = -0.05;
    private double downPosition = 0.5;
    private double midPosition = 0.15;
    private double targetPosition = downPosition;
    private final PositionVoltage m_positionRequest = new PositionVoltage(upPosition);

    public IntakePivot() {
        m_motor = new TalonFX(6);
        m_encoder = new CANcoder(13);

        CANcoderConfiguration encoderconfigs = new CANcoderConfiguration();
        encoderconfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        TalonFXConfiguration motorconfigs = new TalonFXConfiguration();
        motorconfigs.Feedback.RotorToSensorRatio = 1.0;
        motorconfigs.Feedback.SensorToMechanismRatio = 1.0; // TODO: fix
        motorconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorconfigs.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
        motorconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // motorconfigs.Slot0.kP = 10;
        // motorconfigs.Slot0.kV = 50;
        // motorconfigs.Slot0.kG = -2;
        motorconfigs.Slot0.kP = 6.0;
        motorconfigs.Slot0.kD = 0.125;
        motorconfigs.Slot0.kA = 1.24;
        motorconfigs.Slot0.kV = 6.00;
        motorconfigs.Slot0.kG = -.5;
        motorconfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // motorconfigs.Slot0.GravityArmPositionOffset = -0.5; // TODO: fix

        m_motor.getConfigurator().apply(motorconfigs);
    }

    private enum ControlMode {
        kOpenLoop,
        kPID
    }

    public double getAngle() {
	    return m_encoder.getPosition().getValueAsDouble();
    }

    public void intakeUp() {
        m_motor.setControl(m_positionRequest.withPosition(upPosition));
        targetPosition = upPosition;
    }

    public void intakeDown() {
        m_motor.setControl(m_positionRequest.withPosition(downPosition));
        targetPosition = downPosition;
    }

    public void intakeMid() {
        m_motor.setControl(m_positionRequest.withPosition(midPosition));
        targetPosition = midPosition;
    }

    public void setOutputVoltage(double OutputVoltage) {
	    m_controlMode = ControlMode.kOpenLoop;
	    m_demand = OutputVoltage;
        m_motor.setControl(new DutyCycleOut(m_demand));
    }

    public void stop() {
	    setOutputVoltage(0);
    }

    @Override
    public void periodic() {
        // // Determine output voltage
        // double outputVoltage = 0;
        double Angle = getAngle();
        SmartDashboard.putNumber("Intake Pivot Angle", Angle);
        SmartDashboard.putNumber("Intake Pivot Angle From Motor", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Pivot Angle Degrees", Angle*180);
        SmartDashboard.putNumber("Intake Pivot Target", targetPosition);
        if(!isOnTarget(targetPosition, .05)){
            m_motor.setControl(m_positionRequest.withPosition(targetPosition));
        } else {
            stop();
        }
    }

    public boolean isOnTarget(double target, double toleranceDegrees) {
        double currentPosition = getAngle();
        double error = Math.abs(currentPosition - target);
        SmartDashboard.putBoolean("Pivot On Target", error < toleranceDegrees);
        SmartDashboard.putNumber("Pivot isOnTarget Current Position", currentPosition);
        SmartDashboard.putNumber("Pivot isOnTaget Error", error);
        return error < toleranceDegrees;
    }

}
