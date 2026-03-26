// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.constant.DirectMethodHandleDesc;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Localization.Vision;
import frc.robot.commands.ShooterCommand;

import frc.robot.subsystems.IntakePivot;
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Shooter shooter = new Shooter();
    //public final Turret turret = new Turret();
    public final Transfer transfer = new Transfer();
    public final Hopper hopper = new Hopper();
    public final Intake intake = new Intake();
    public double distanceToHub;
    public final Vision limelight = new Vision(drivetrain::addVisionMeasurement, () -> drivetrain.getState().Pose);

    public final IntakePivot intakepivot = new IntakePivot();
    public RobotContainer() {
        configureBindings();
        configurePathPlannerCommands();
        drivetrain.configureAutoBuilder();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.rightTrigger().whileTrue(shooter.runShooterCommand());

        // joystick.rightBumper().onTrue(shooter.increaseDistance());
        // joystick.leftBumper().onTrue(shooter.decreaseDistance());

        
        drivetrain.registerTelemetry(logger::telemeterize);
        
        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)));
        joystick.rightTrigger().whileTrue(new ShooterCommand(shooter, shooter.hood, transfer, hopper, limelight, limelight.turret, !limelight.inAllianceZone));
        joystick.leftTrigger().whileTrue(intake.test_forwardsCommand());
        joystick.x().whileTrue(intakepivot.runOnce(() -> intakepivot.intakeUp()));
        joystick.b().whileTrue(intakepivot.runOnce(() -> intakepivot.intakeDown()));
        joystick.a().whileTrue(IntakeFeedCommand());
        // joystick.leftTrigger().whileTrue(new ShooterCommand(shooter, shooter.hood, transfer, hopper, limelight, true));

       // joystick.b().whileTrue(turret.runOnce(() -> turret.setTargetAngle(new Rotation2d(0))));
        // joystick.a().whileTrue(turret.runOnce(() -> turret.setTargetAngle(new Rotation2d(Math.PI / 2))));

        // joystick.x().whileTrue(shooter.hood.increaseAngleCommand());
        // joystick.y().whileTrue(shooter.hood.decreaseAngleCommand());

        joystick.povRight().whileTrue(limelight.runOnce(() -> limelight.ResetPoseCommand()));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("FirstShoot");
    }

    public void configurePathPlannerCommands(){
        NamedCommands.registerCommand("DeployIntake", intakepivot.runOnce(() -> intakepivot.intakeDown()));
        NamedCommands.registerCommand("RunIntake", intake.test_forwardsCommand());
        NamedCommands.registerCommand("RunShooter", new ShooterCommand(shooter, shooter.hood, transfer, hopper, limelight, limelight.turret, false));
        NamedCommands.registerCommand("Feed", IntakeFeedCommand());
    }

    public Command IntakeFeedCommand() {
        return new ParallelCommandGroup(intakepivot.runOnce(() -> intakepivot.intakeMid()),
                                        intake.test_forwardsCommand());
    }
}