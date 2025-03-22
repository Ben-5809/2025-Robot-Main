// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.libaries.LimelightHelpers;
import frc.robot.libaries.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;


public class DriveToTag extends Command {
  private final VisionSubsystem vision;
  private final CommandSwerveDrivetrain drivetrain;
  private final boolean goLeft;
  private final String limelight;

  private Pose2d targetPosition;
  private SwerveRequest.ApplyRobotSpeeds visionRequest;
  private ChassisSpeeds visionSpeeds;

  public DriveToTag(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, boolean goLeft, String LimelightName) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.goLeft = goLeft;
    this.limelight = LimelightName;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    targetPosition = vision.getTargetPos(limelight, goLeft);
  }

  @Override
  public void execute() {
    // Tells the limelight where we are on the field
    LimelightHelpers.SetRobotOrientation(VisionConstants.LIMELIGHT_NAMES[0],
        drivetrain.getRotation3d().getZ() * 57.2958, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(VisionConstants.LIMELIGHT_NAMES[1],
        drivetrain.getRotation3d().getZ() * 57.2958, 0, 0, 0, 0, 0);
    AngularVelocity gyroRate = drivetrain.getPigeon2().getAngularVelocityZWorld().getValue();

    Optional<PoseEstimate> estimatedPose = vision.determinePoseEstimate(gyroRate);
    if (estimatedPose.isPresent()) {
      drivetrain.addVisionMeasurement(estimatedPose.get().pose, estimatedPose.get().timestampSeconds);
    }

    visionSpeeds = vision.calculateChassisSpeeds(targetPosition, drivetrain.getState().Pose);
    drivetrain.setControl(visionRequest.withSpeeds(visionSpeeds));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(null);
  }
}

