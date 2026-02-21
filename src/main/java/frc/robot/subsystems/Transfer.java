package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
  private final TalonFX m_transfer;
  //placeholder Transfer MotorID
  public Transfer() {
    m_transfer = new TalonFX(7);
  }
  public void transfer() {
	m_transfer.setVoltage(4);
  }
  public void reverse_transfer () {
  m_transfer.setVoltage(-6);

  }
  public void transferPercent(double volt){
    m_transfer.setVoltage(volt);
  }
  public void stop() {
    m_transfer.stopMotor();
  }
  public Command test_forwardsCommand() {
    return Commands.runEnd(
      ()-> this.transfer(),
      this::stop,
      this);
  }
    public Command test_reverseCommand() {
      return Commands.runEnd(
      ()-> this.reverse_transfer(),
      this::stop,
      this);
  }
  public Command transferPercentCommand(double volt){
      return Commands.runEnd(
      ()-> this.transferPercent(volt),
      this::stop,
      this);	
  }
}