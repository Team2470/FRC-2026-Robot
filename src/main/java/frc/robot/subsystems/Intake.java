package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  //change id
  private final TalonFX m_intake;
  public Intake() {
    m_intake = new TalonFX(0);
  }


  public void intake() {
	m_intake.setVoltage(4);
  }
  public void reverse_intake () {
  m_intake.setVoltage(-6);

  }
  public void intakePercet(double volt){
    m_intake.setVoltage(volt);
  }
  public void stop() {
    m_intake.stopMotor();
  }
  public Command test_forwardsCommand() {
    return Commands.runEnd(
      ()-> this.intake(),
      this::stop,
      this);
  }
    public Command test_reverseCommand() {
      return Commands.runEnd(
      ()-> this.reverse_intake(),
      this::stop,
      this);
  }
  public Command intakePercentCommand(double volt){
      return Commands.runEnd(
      ()-> this.intakePercet(volt),
      this::stop,
      this);	
  }
}

