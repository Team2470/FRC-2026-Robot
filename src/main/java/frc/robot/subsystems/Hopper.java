package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.hopperConstants;

public class Hopper extends SubsystemBase {
  private final TalonFX m_hopper;
  //placeholder Hopper MotorID
  public Hopper() {
    m_hopper = new TalonFX(hopperConstants.HOPPER_DEVICE_ID);
  }
  public void run() {
	m_hopper.setVoltage(hopperConstants.RUN_HOPPER_VOLTAGE);
  }
  public void reverse_hopper () {
  m_hopper.setVoltage(hopperConstants.REVERSE_HOPPER_VOLTAGE);

  }
  public void run(double volt){
    m_hopper.setVoltage(volt);
  }
  public void stop() {
    m_hopper.stopMotor();
  }
  public Command test_forwardsCommand() {
    return Commands.runEnd(
      ()-> this.run(),
      this::stop,
      this);
  }
    public Command test_reverseCommand() {
      return Commands.runEnd(
      ()-> this.reverse_hopper(),
      this::stop,
      this);
  }
  public Command runCommand(double volt){
      return Commands.runEnd(
      ()-> this.run(volt),
      this::stop,
      this);
  }
}