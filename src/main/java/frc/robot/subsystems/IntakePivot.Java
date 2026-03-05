package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
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
    private ControlMode m_controlMode;
    private double uplimit = 50;
    
    public IntakePivot() {
        m_motor = new TalonFX(6);
        m_encoder = new CANcoder(0);
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
    public Command intakeUp() {
        return new SequentialCommandGroup(
        new InstantCommand(()-> uplimit = 118),
        openLoopCommand(4).until(()->getAngle() > 60),
        openLoopCommand(2).until(()-> getAngle() > 115)
        );
    }

    public Command intakeDown() {
	return new SequentialCommandGroup(
		openLoopCommand(-3).until(()-> getAngle() < 10),
		openLoopCommand(-0.25)
	);
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
                // outputVoltage = m_pidController.calculate(getAngle(), m_demand);

                break;
            default:
                // What happened!?
                break;
        }

        if (Angle <= 10 && outputVoltage < 0 || Angle >= uplimit && outputVoltage > 0 ){
	    outputVoltage = 0;
	    }
        
	    m_motor.setVoltage(outputVoltage);
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