package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
      //change id
  private final TalonFX m_feeder;
  public Feeder() {
    m_feeder = new TalonFX(0);
  }


  public void run() {
	m_feeder.setVoltage(4);
  }
  public void reverse_feeder () {
    m_feeder.setVoltage(-6);

  }
  public void feederPercent(double volt){
    m_feeder.setVoltage(volt);
  }
  public void stop() {
    m_feeder.stopMotor();
  }
  public Command test_forwardsCommand() {
    return Commands.runEnd(
      ()-> this.run(),
      this::stop,
      this);
  }
    public Command test_reverseCommand() {
      return Commands.runEnd(
      ()-> this.reverse_feeder(),
      this::stop,
      this);
  }
  public Command feederPercentCommand(double volt){
      return Commands.runEnd(
      ()-> this.feederPercent(volt),
      this::stop,
      this);
  }
}
