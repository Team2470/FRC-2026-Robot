package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
    //change id
    private final TalonFX m_motor;
    private final CANcoder m_encoder;
    private double m_demand;
    
    public IntakePivot() {
        m_motor = new TalonFX(0);
        m_encoder = new CANcoder(0);
    }
    }
    private enum ControlMode {
        kOpenLoop,
        kPID
    }

    public double getAngle() {
	    return m_encoder.getAbsolutePosition();
    }

    @Override
    public void periodic() {
        // Determine output voltage
        double outputVoltage = 0;
        double Angle = getAngle();
        switch (m_controlMode) {
            case kOpenLoop:
                // Do openloop stuff here
                outputVoltage = m_demand;
                break;

            case kPID:
                // Do PID stuff
                outputVoltage = m_pidController.calculate(getAngle(), m_demand);

                break;
            default:
                // What happened!?
                break;
            
	        m_motor.setVoltage(outputVoltage);
        }

    }