// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveToTag extends Command {
  private final VisionSubsystem vision;
  private final CommandSwerveDrivetrain drivetrain;
  private final boolean goLeft;
  private final String limelight;

  private Pose2d targetPosition;
  private SwerveRequest.ApplyRobotSpeeds visionRequest = new SwerveRequest.ApplyRobotSpeeds();
  private ChassisSpeeds visionSpeeds;
  private ChassisSpeeds zero = new ChassisSpeeds(0,0,0);
  private SwerveRequest.ApplyRobotSpeeds zeroRequest = new SwerveRequest.ApplyRobotSpeeds();

  public DriveToTag(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, boolean goLeft, String LimelightName) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.goLeft = goLeft;
    this.limelight = LimelightName;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    targetPosition = vision.getTargetPos(goLeft);
  }

  @Override
  public void execute() {
    visionSpeeds = vision.calculateChassisSpeeds(targetPosition, drivetrain.getState().Pose);
    drivetrain.setControl(visionRequest.withSpeeds(visionSpeeds));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(zeroRequest.withSpeeds(zero));
  }
}
*/
