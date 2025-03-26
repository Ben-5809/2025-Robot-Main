// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.Stripper;

public class IntakeBall extends Command {
  private CoralEndEffector frontWheel;
  private Stripper backWheel;
  private double frontVoltage;
  private double backVoltage;

  public IntakeBall(CoralEndEffector frontWheel, Stripper backWheel, double frontVoltage, double backVoltage) {
    this.frontWheel = frontWheel;
    this.backWheel = backWheel;
    this.frontVoltage = frontVoltage;
    this.backVoltage = backVoltage;
    
    addRequirements(frontWheel);
    addRequirements(backWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    frontWheel.setMotorVoltage(frontVoltage);
    backWheel.setMotorVoltage(backVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
  
