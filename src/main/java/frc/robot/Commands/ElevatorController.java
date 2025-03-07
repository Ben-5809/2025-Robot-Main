// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.LEDsub;
import frc.robot.Commands.EndEffectorVoltage;

public class ElevatorController extends Command {
  //Declarations of the elevator sub
  private final ElevatorSub elevatorSub;
  private final LEDsub ledSub;
  private final int[] colorApplied;
  private final double height;;

  public ElevatorController(ElevatorSub elevatorSub, LEDsub ledSub, int[] colorApplied, double height) {
    //Connects the elevator sub to the passthrough in robotContainer
    this.elevatorSub = elevatorSub;
    this.ledSub = ledSub;
    this.colorApplied = colorApplied;
    this.height = height;

    //Cancels other command if a command 
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets the motion profile to the set position
    elevatorSub.setPosition(height);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSub.setLED(colorApplied);
    elevatorSub.applyVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevatorSub.getElevatorPosLeft() > (height - 0.1) &&  elevatorSub.getElevatorPosLeft() < (height + 0.1)) {
      return true;
      
    }
    else { 
      return false;
    }
  }
}
