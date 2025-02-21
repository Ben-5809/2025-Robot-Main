// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ElevatorApplyVoltage extends Command {
  //ElevatorSub declaration
  private final ElevatorSub elevatorSub;
  private final double elevatorVoltage;

  public ElevatorApplyVoltage(ElevatorSub elevatorSub, double elevatorVoltage) {
    this.elevatorSub = elevatorSub;
    this.elevatorVoltage = elevatorVoltage;

    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSub.applyVoltage(elevatorVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSub.applyVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
