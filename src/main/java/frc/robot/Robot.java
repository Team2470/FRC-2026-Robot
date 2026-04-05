// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Localization.LimelightHelpers;
import frc.robot.subsystems.Localization.Vision;
import gg.questnav.questnav.QuestNav;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private double m_lastLimelightPrintTime = 0.0;
    public double distanceToHub;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        FollowPathCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putBoolean("isBlueAlliance", Constants.isBlueAlliance());
        SmartDashboard.putNumber("Hub X", Constants.HUB_LOCATION.getX());
        SmartDashboard.putNumber("Hub Y", Constants.HUB_LOCATION.getY());
        SmartDashboard.putNumber("Pass Left X", Constants.LEFT_PASS_LOCATION.getX());
        SmartDashboard.putNumber("Pass Left Y", Constants.LEFT_PASS_LOCATION.getY());
        SmartDashboard.putNumber("Pass Right X", Constants.RIGHT_PASS_LOCATION.getX());
        SmartDashboard.putNumber("Pass Right Y", Constants.RIGHT_PASS_LOCATION.getY());
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
        SmartDashboard.putBoolean("isBlueAlliance", Constants.isBlueAlliance());
        SmartDashboard.putNumber("Hub X", Constants.HUB_LOCATION.getX());
        SmartDashboard.putNumber("Hub Y", Constants.HUB_LOCATION.getY());
        SmartDashboard.putNumber("Pass Left X", Constants.LEFT_PASS_LOCATION.getX());
        SmartDashboard.putNumber("Pass Left Y", Constants.LEFT_PASS_LOCATION.getY());
        SmartDashboard.putNumber("Pass Right X", Constants.RIGHT_PASS_LOCATION.getX());
        SmartDashboard.putNumber("Pass Right Y", Constants.RIGHT_PASS_LOCATION.getY());
    }

    @Override
    public void autonomousPeriodic() {
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