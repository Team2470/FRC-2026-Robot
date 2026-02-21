// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.LimelightHelpers;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private double m_lastLimelightPrintTime = 0.0;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        if (RobotBase.isSimulation()) {
            var nt = NetworkTableInstance.getDefault();
            nt.stopClient();
            nt.startClient4("sim");
            // nt.setServer(new String[] {"172.28.0.1", "limelight.local"}); // Windows
            nt.setServer(new String[] { "172.29.0.1", "limelight.local" }); // Mac
            nt.startDSClient();
        }
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        double now = Timer.getFPGATimestamp();
        if (now - m_lastLimelightPrintTime < (1.0 / 24.0)) {
            return;
        }
        m_lastLimelightPrintTime = now;

        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        String limelightName = "limelight-left";
        var poseEstimate = isRed
                ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (poseEstimate == null) {
            System.out.println("Limelight pose estimate unavailable (" + limelightName + ").");
            return;
        }
        var pose = poseEstimate.pose;

        var poseX = pose.getX();
        var poseY = pose.getY();
        var rotation = pose.getRotation().getDegrees();

        if (poseX > 0 || poseY > 0 || rotation > 0) {

            System.out.printf(
                    "Limelight pose estimate: x=%.2f y=%.2f rot=%.1f deg%n",
                    pose.getX(),
                    pose.getY(),
                    pose.getRotation().getDegrees());
        }
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
