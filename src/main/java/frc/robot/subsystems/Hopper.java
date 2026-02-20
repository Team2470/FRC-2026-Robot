package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
      //change id
  private final TalonFX m_hopper;
  public Hopper() {
    m_hopper = new TalonFX(0);
  }


  public void run() {
	m_hopper.setVoltage(4);
  }
  public void reverse_hopper () {
  m_hopper.setVoltage(-6);

  }
  public void hopperPercent(double volt){
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
  public Command hopperPercentCommand(double volt){
      return Commands.runEnd(
      ()-> this.hopperPercent(volt),
      this::stop,
      this);
  }
}
