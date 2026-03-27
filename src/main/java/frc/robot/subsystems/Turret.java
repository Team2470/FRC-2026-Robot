package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class Turret extends SubsystemBase {
    private final TalonFX m_turretMotor             = new TalonFX(shooterConstants.TURRET_DEVICE_ID);
    private final MotionMagicVoltage m_mmRequest    = new MotionMagicVoltage(0);
    private final CANcoder m_turretCanCoder = new CANcoder(12);


    // Adjust based on your physical gear ratio (e.g., 100:1)
    private final double GEAR_RATIO = shooterConstants.TURRET_GEAR_RATIO;
    private final double ENCODERATIO = shooterConstants.TURRET_ENCODER_RATIO;
    private final double MAX_TURRET_ROTATIONS = shooterConstants.MAX_TURRET_ROTATIONS.getRotations();
    private final double MIN_TURRET_ROTATIONS = shooterConstants.MIN_TURRET_ROTATIONS.getRotations();
    public Rotation2d turretAngle;


    public Turret() {
        TalonFXConfiguration config                     = new TalonFXConfiguration();
        config.Slot0.kP                                 = shooterConstants.TURRET_KP;
        config.Slot0.kI                                 = shooterConstants.TURRET_KI;
        config.Slot0.kD                                 = shooterConstants.TURRET_KD;
        config.Slot0.kV                                 = shooterConstants.TURRET_KV;
        config.Slot0.kS                                 = shooterConstants.TURRET_KS;
        config.MotionMagic.MotionMagicCruiseVelocity    = shooterConstants.TURRET_MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration      = shooterConstants.TURRET_MOTION_MAGIC_ACCELERACTIION;
        m_turretMotor.getConfigurator().apply(config);

    }

    public void periodic(){
        SmartDashboard.putNumber("Angle", getTurretAngle().getDegrees());
        if (DriverStation.isAutonomous()){
            setTargetAngle(new Rotation2d(-0.25)); // In Auto, we shoot from a position where the turret must point to the robot's left
        }
    }

    public Rotation2d getTurretAngle(){
        Rotation2d angle = Rotation2d.fromRadians(Math.PI/2);
        StatusSignal<Angle> angleSignal = m_turretCanCoder.getPosition();
        angle = new Rotation2d(angleSignal.getValue() );
        angle = new Rotation2d(angle.getRadians()/ENCODERATIO);
        // angle = angle.plus(Rotation2d.fromRotations(.3)); 
        SmartDashboard.putNumber("Current_Turret_Angle", angle.getRotations());
        return angle;
    }

    /**
     * Set turret position.
     * @param robotRelativeAngle Angle relative to the robot's front.
     */
    public void setTargetAngle(Rotation2d robotRelativeAngle) {
        double rotations = robotRelativeAngle.getRotations() * GEAR_RATIO;
        // if (rotations < MIN_TURRET_ROTATIONS) {
        //     rotations = MIN_TURRET_ROTATIONS;
        // } else if (rotations > MAX_TURRET_ROTATIONS) {
        //     rotations = MAX_TURRET_ROTATIONS;
        // }
        // if (getTurretAngle().getDegrees() < robotRelativeAngle.getDegrees()) {
        //     m_mmRequest.FeedForward = 0.5;
        // } else {
        //     m_mmRequest.FeedForward = 0.8;
        //     // rotations = rotations - 1.0;
        // }

        m_mmRequest.FeedForward = 0.8;
        m_turretMotor.setControl(m_mmRequest.withPosition(rotations));
        SmartDashboard.putNumber("setTargetAngle rotations", rotations);
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
            if(turretAngle.getRotations() < MIN_TURRET_ROTATIONS){
                turretAngle = Rotation2d.fromRotations(MIN_TURRET_ROTATIONS);
            } else if (turretAngle.getRotations() > MAX_TURRET_ROTATIONS){
                turretAngle = Rotation2d.fromRotations(MAX_TURRET_ROTATIONS);
            }
        },
        () -> { this.setTargetAngle(turretAngle);}, this);
    }
}