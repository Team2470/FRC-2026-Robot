package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class Hood extends SubsystemBase {
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(30).per(Second);
    private static final double kMinPosition = 0.01;
    private static final double kMaxPosition = 0.99;
    private static final double kPositionTolerance = 0.01;
    private int leftChannel = 0;
    private int rightChannel = 1;
    private final Servo leftServo;
    private final Servo rightServo;
    private double leftCurrentPosition = 0.0;
    private double rightCurrentPosition = 0.0;
    private double currentAngle = 25;
    private double targetAngle = 25;
    private double leftTargetPosition = 0.0;
    private double rightTargetPosition = 0.0;
    private Time lastUpdateTime = Seconds.of(0);

    public Hood() {
        leftServo = new Servo(leftChannel);
        rightServo = new Servo(rightChannel);
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setAngle(currentAngle);
        SmartDashboard.putData(this);
    }
    
    /** Expects an angle between 25 and 65*/
    public void setAngle (double angle) {
        double clamped      = Math.max(shooterConstants.MIN_HOOD_ANGLE,
                                Math.min(shooterConstants.MAX_HOOD_ANGLE, angle));
        
        double angleAsRadians = Math.toRadians((clamped) + 42.23); // mirror clamped angle and add constant angle (from CAD) for trig
        // Calculation derived from CAD model to convert angle to actuator extention  
        double leftLength = (Math.sqrt((6.63*6.63) + (5.078*5.078) - (2*6.63*5.078 * Math.cos(angleAsRadians))));
        double leftPosition = (leftLength - 6.61)/(10.48-6.61); // Convert total length of actuator to percentage of extention
        double rightLength = Math.sqrt(103.25-99.727*(Math.cos(Math.acos((69.756 - leftLength*leftLength)/ 67.3444) - 0.243)));
        double rightPosition = (rightLength - 6.61)/(10.48-6.61);
        final double leftClampedPosition = MathUtil.clamp(leftPosition, kMinPosition, kMaxPosition);
        final double rightClampedPosition = MathUtil.clamp(rightPosition, kMinPosition, kMaxPosition);
        leftServo.set(leftClampedPosition);
        rightServo.set(rightClampedPosition);
        leftTargetPosition = leftClampedPosition;
        rightTargetPosition = rightClampedPosition;
        // targetAngle = Math.toDegrees(angleAsRadians);
    }

    public void increaseAngle () {
        targetAngle = targetAngle + 5;
    }
    
    public void decreaseAngle () {
        targetAngle = targetAngle - 5;
    }
    /* public void extendActuator() {
        setPosition(currentPosition + 0.05);
    }

    public void retractActuator() {
        setPosition(currentPosition - 0.05);
    } */

    /** Expects a position between 0.0 and 1.0 */
    public Command angleCommand(double angle) {
        return runOnce(() -> setAngle(angle))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }
    
    public Command increaseAngleCommand() {
        return runOnce(() -> increaseAngle())
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public Command decreaseAngleCommand() {
        return runOnce(() -> decreaseAngle())
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }
    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(leftTargetPosition, leftCurrentPosition, kPositionTolerance) && MathUtil.isNear(rightTargetPosition, rightCurrentPosition, kPositionTolerance) ;
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            leftCurrentPosition = leftTargetPosition;
            rightCurrentPosition = rightTargetPosition;
            currentAngle = targetAngle;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        leftCurrentPosition = leftTargetPosition > leftCurrentPosition
            ? Math.min(leftTargetPosition, leftCurrentPosition + maxPercentageTraveled)
            : Math.max(leftTargetPosition, leftCurrentPosition - maxPercentageTraveled);
        rightCurrentPosition = rightTargetPosition > rightCurrentPosition
            ? Math.min(rightTargetPosition, rightCurrentPosition + maxPercentageTraveled)
            : Math.max(rightTargetPosition, rightCurrentPosition - maxPercentageTraveled);
    }


    @Override
    public void periodic() {
        updateCurrentPosition();
        setAngle(targetAngle);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Left Current Position", () -> leftCurrentPosition, null);
        builder.addDoubleProperty("Left Target Position", () -> leftTargetPosition, value -> setAngle(value));
        builder.addDoubleProperty("Right Current Position", () -> rightCurrentPosition, null);
        builder.addDoubleProperty("Right Target Position", () -> rightTargetPosition, value -> setAngle(value));
        builder.addDoubleProperty("Current Angle", () -> currentAngle, null);
        builder.addDoubleProperty("Target Angle", () -> targetAngle, value -> setAngle(value));
    }
}