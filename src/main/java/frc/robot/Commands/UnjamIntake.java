// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.LEDsub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UnjamIntake extends Command {
    private CoralEndEffector coralEndEffector;
  private LEDsub ledSub;
  private int[] colorApplied;
  private double motorVoltage;

  public UnjamIntake(CoralEndEffector coralEndEffector, LEDsub ledSub, int[] colorApplied, double motorVoltage) {
    this.coralEndEffector = coralEndEffector;
    this.ledSub = ledSub;
    this.colorApplied = colorApplied;
    this.motorVoltage = motorVoltage;  

    addRequirements(coralEndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSub.setLED(Constants.CANdleCons.saturatedRed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralEndEffector.setMotorVoltage(motorVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSub.setLED(colorApplied);
    coralEndEffector.setMotorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
