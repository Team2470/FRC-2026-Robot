package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.transferConstants;

public class Transfer extends SubsystemBase {
  private final TalonFX m_transfer;
  //placeholder Transfer MotorID
  public Transfer() {
    m_transfer = new TalonFX(transferConstants.TRANSFER_DEVICE_ID);
  }
  public void transfer() {
	m_transfer.setVoltage(transferConstants.RUN_TRANSFER_VOLTAGE);
  }
  public void reverse_transfer () {
  m_transfer.setVoltage(transferConstants.REVERSE_TRANSFER_VOLTAGE);

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