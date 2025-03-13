// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.net.PortForwarder;

import com.ctre.phoenix6.hardware.*;


import com.ctre.phoenix6.SignalLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Pigeon2 gyro = new Pigeon2(51);

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    gyro.setYaw(180);

    m_robotContainer.setMegaTag2(false);
    /* 
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, Constants.VisionConstants.LIMELIGHT_NAMES[1], port);
    }
    */
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.UpdateVision().schedule();

    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    SignalLogger.start();
  }

  @Override
  public void autonomousInit() {

    m_robotContainer.setMegaTag2(true);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.setMegaTag2(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
