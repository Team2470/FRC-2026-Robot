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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.IntakePivotConstants;

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
    private double upPosition = -0.35;
    private double downPosition = 0.6;
    private double midPosition = 0;
    private final PositionVoltage m_positionRequest = new PositionVoltage(upPosition);
    
    public IntakePivot() {
        m_motor = new TalonFX(6);
        m_encoder = new CANcoder(13);

        CANcoderConfiguration encoderconfigs = new CANcoderConfiguration();
        encoderconfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        TalonFXConfiguration motorconfigs = new TalonFXConfiguration();
        motorconfigs.Feedback.RotorToSensorRatio = 1;
        motorconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorconfigs.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
        motorconfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;        
        motorconfigs.Slot0.kP = 7.5;
        motorconfigs.Slot0.kV = 50;
        motorconfigs.Slot0.kG = 2;
        motorconfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        m_motor.getConfigurator().apply(motorconfigs);
    }
    
    private enum ControlMode {
        kOpenLoop,
        kPID
    }

    public double getAngle() {
	    return m_encoder.getAbsolutePosition().getValueAsDouble();
    }

    
    private Command openLoopCommand(double OutputVoltage) {


        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return Commands.runEnd(
            () -> this.setOutputVoltage(OutputVoltage), this::stop, this);

    }

    // private Command openLoopCommand(double OutputVoltage) {
    //     return openLoopCommand(()-> OutputVoltage);
    // }
    public void intakeUp() {
        m_motor.setControl(m_positionRequest.withPosition(upPosition));
    }

    public void intakeDown() {
        m_motor.setControl(m_positionRequest.withPosition(downPosition));
    }

    public void intakeMid() {
        m_motor.setControl(m_positionRequest.withPosition(midPosition));
    }

    public void setOutputVoltage(double OutputVoltage) {
	m_controlMode = ControlMode.kOpenLoop;
	m_demand = OutputVoltage;
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

        // switch (m_controlMode) {
        //     case kOpenLoop:
        //         // Do openloop stuff here
        //         outputVoltage = m_demand;
        //         break;

        //     case kPID:
        //         // Do PID stuff
        //         // outputVoltage = m_pidController.calculate(getAngle(), m_demand);

        //         break;
        //     default:
        //         // What happened!?
        //         break;
        // }

        // if (Angle >= 0.4 && outputVoltage < 0 || Angle <= 0 && outputVoltage > 0 ){
	    // outputVoltage = 0;
	    // }
        
	    // m_motor.setVoltage(outputVoltage);
    }
    // If we need:

          // public boolean isAtRetractedLimit(){
    //     return (getAngle() >= 90);
    // }

    // public boolean isAtExtentedLimit(){
    //     return(getAngle() <= 1);
    // }

        
    // public void setBrakeMode (boolean enabled) {
    //     if(enabled) {
    //         m_motor.setIdleMode(IdleMode.kBrake);
            
    //     } else {
    //         m_motor.setIdleMode(IdleMode.kCoast);
    //     }
    // }
}