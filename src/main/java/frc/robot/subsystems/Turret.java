package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class Turret extends SubsystemBase {
    private final TalonFX m_turretMotor             = new TalonFX(shooterConstants.TURRET_DEVICE_ID);
    private final MotionMagicVoltage m_mmRequest    = new MotionMagicVoltage(0);
    private Rotation2d turretAngle = Rotation2d.fromRadians(0);
    private final CANcoder m_turretCanCoder = new CANcoder(12);

    // Adjust based on your physical gear ratio (e.g., 100:1)
    private final double GEAR_RATIO = shooterConstants.TURRET_GEAR_RATIO;

    public Turret() {
        TalonFXConfiguration config                     = new TalonFXConfiguration();
        config.Slot0.kP                                 = shooterConstants.TURRET_KP;
        config.Slot0.kI                                 = shooterConstants.TURRET_KI;
        config.Slot0.kD                                 = shooterConstants.TURRET_KD;
        config.Slot0.kV                                 = shooterConstants.TURRET_KV;
        config.MotionMagic.MotionMagicCruiseVelocity    = shooterConstants.TURRET_MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration      = shooterConstants.TURRET_MOTION_MAGIC_ACCELERACTIION;
        m_turretMotor.getConfigurator().apply(config);
    }

    /**
     * Set turret position.
     * @param robotRelativeAngle Angle relative to the robot's front.
     */
    public void setTargetAngle(Rotation2d robotRelativeAngle) {
        double rotations = robotRelativeAngle.getRotations() * GEAR_RATIO;
        m_turretMotor.setControl(m_mmRequest.withPosition(rotations));
    }

    public boolean isOnTarget(Rotation2d target, double toleranceDegrees) {
        double currentRot = m_turretMotor.getPosition().getValueAsDouble() / GEAR_RATIO;
        double error = Math.abs(currentRot - target.getRotations()) * 360.0;
        return error < toleranceDegrees;
    }
    public Command runTurretCommand(Integer direction){
    return Commands.runEnd(
        () -> {
            this.setTargetAngle(turretAngle.plus(Rotation2d.fromRadians(Math.PI/180 * direction)));
            turretAngle = turretAngle.plus(Rotation2d.fromRadians(Math.PI/180 * direction));
        },
        () -> { this.setTargetAngle(turretAngle);}, this);
    }
}
