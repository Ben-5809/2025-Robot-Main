// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimatorSub;

public class AlignWithVision extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private PoseEstimatorSub poseEstimator;
  
  public AlignWithVision(CommandSwerveDrivetrain drivetrain, PoseEstimatorSub poseEstimator) {
    this.drivetrain = drivetrain;
    this.poseEstimator = poseEstimator;
    
    addRequirements(drivetrain, poseEstimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    


    if (interrupted == false) {
      poseEstimator.setPoseTranslation(poseEstimator.getReefPose().getTranslation());
  }


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
*/
