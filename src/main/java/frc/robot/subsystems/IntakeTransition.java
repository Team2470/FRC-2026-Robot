package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeTransition extends SubsystemBase {
  private final TalonFX m_intaketransition;
    
  public IntakeTransition() {
    m_intaketransition = new TalonFX(0);
  }
  public void intaketransition() {
	m_intaketransition.setVoltage(4);
  }
  public void reverse_intaketransition () {
  m_intaketransition.setVoltage(-6);

  }
  public void intaketransitionPercet(double volt){
    m_intaketransition.setVoltage(volt);
  }
  public void stop() {
    m_intaketransition.stopMotor();
  }
  public Command test_forwardsCommand() {
    return Commands.runEnd(
      ()-> this.intaketransition(),
      this::stop,
      this);
  }
    public Command test_reverseCommand() {
      return Commands.runEnd(
      ()-> this.reverse_intaketransition(),
      this::stop,
      this);
  }
  public Command intaketransitionPercentCommand(double volt){
      return Commands.runEnd(
      ()-> this.intaketransitionPercet(volt),
      this::stop,
      this);	
  }
}
