package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.intakePivotConstants;


public class IntakePivot extends SubsystemBase {
    //change id
    private final TalonFX m_motor;
    private final CANcoder m_encoder;
    private double m_demand;
    // 50 is a placeholder.
    private double uplimit = 50;
    
    public IntakePivot() {
        m_motor = new TalonFX(0);
        m_encoder = new CANcoder(0);
    }

    private enum ControlMode {
        kOpenLoop,
        kPID
    }
    

    private ControlMode m_controlMode = ControlMode.kOpenLoop;
    public double getAngle() {
        return m_encoder.getPosition().getValueAsDouble();

    }
    private final PIDController m_pidController = new PIDController(intakePivotConstants.PIVOT_KP, intakePivotConstants.PIVOT_KI, intakePivotConstants.PIVOT_KD);

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
            
        }
        //I assume this makes sure that this makes sure the robot stops from breaking itself, the <= 10 is a place holder for now
        if (Angle <= 10 && outputVoltage < 0 || Angle >= uplimit && outputVoltage > 0 ) {
            outputVoltage = 0;
            }
	    m_motor.setVoltage(outputVoltage);
    }

        public double getErrorAngle() {
            if (m_controlMode == ControlMode.kPID){
            return m_pidController.getPositionError();
            }
            return 0;
        }

        public void setOutputVoltage(double OutputVoltage) {
            m_controlMode = ControlMode.kOpenLoop;
            m_demand = OutputVoltage;
        }

        public void setPIDSetpoint(double angleDegrees) {
            m_controlMode = ControlMode.kPID;
            m_demand = angleDegrees;
        }

        public void stop() {
            setOutputVoltage(0);
        }
        /*  This is broken and I don't know if we need a brakemode yet
        public void setBrakeMode (boolean enabled) {
            if(enabled) {
                m_motor.setIdleMode(IdleMode.kBrake);
                
            } else {
                m_motor.setIdleMode(IdleMode.kCoast);
            }
        } */

        /**
        * Example command factory method.
        *
        * @return a command
        */
        private Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {


            // Inline construction of command goes here.
            // Subsystem::RunOnce implicitly requires `this` subsystem.
            return Commands.runEnd(
                () -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);

        }

        private Command openLoopCommand(double OutputVoltage) {
            return openLoopCommand(()-> OutputVoltage);
        }
        

        //I have no idea how this works.
        public Command stowCommand() {
            return new SequentialCommandGroup(
            new InstantCommand(()-> uplimit = 118),
            openLoopCommand(()-> 4).until(()->getAngle() > 60),
            openLoopCommand(()-> 2).until(()-> getAngle() > 115)
            // openLoopCommand(0.75).until(()-> getAngle() > 118)
            );
        }

        public Command intakeLocation() {
            return new SequentialCommandGroup(
            new InstantCommand(()-> uplimit = 55),
            openLoopCommand(()-> 6)
            );
        }

        public Command deploy() {
            return new SequentialCommandGroup(
                openLoopCommand(()-> -3).until(()-> getAngle() < 10),
                openLoopCommand(-0.25)
            );
        }



        public Command goToAngle(DoubleSupplier angleSupplier){
            return Commands.runEnd(
            () -> this.setPIDSetpoint(angleSupplier.getAsDouble()), this::stop, this);
        }

        public Command goToAngle(double angleDegrees) {
            return goToAngle(()-> angleDegrees);
        }
    }

    