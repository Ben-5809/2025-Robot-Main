// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlign extends Command {
  private VisionSubsystem vision;
  private CommandSwerveDrivetrain drivetrain;
  private boolean goLeft;
  private String Limelight;

  private Pose2d targetPose;
  private ChassisSpeeds zero = new ChassisSpeeds(0, 0, 0);
  private SwerveRequest.ApplyRobotSpeeds zeroRequest = new SwerveRequest.ApplyRobotSpeeds();

  public AutoAlign(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, boolean goLeft, String Limelight) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.goLeft = goLeft;
    this.Limelight = Limelight;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = vision.getTargetPos(goLeft);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vision.getAlignmentSpeeds(targetPose, drivetrain);
    vision.autoAlignChassisSpeeds(targetPose, drivetrain);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(zeroRequest.withSpeeds(zero));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.isAligned(drivetrain);
  }
}
*/