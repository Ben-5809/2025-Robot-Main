// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.VisionSubsystem;

public class AutoIntake extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final VisionSubsystem vision;
  private final CoralEndEffector intake;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSup;

  double translation;
  double rotation;
  double strafe;

  private boolean coralSeen;
  private double coralYaw;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AutoIntake(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, CoralEndEffector intake, DoubleSupplier translationSup, DoubleSupplier strafeSupplier, DoubleSupplier rotationSup) { //Command constructor
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.intake = intake;

    this.translationSup = translationSup;
    this.strafeSupplier = strafeSupplier;
    this.rotationSup = rotationSup;

    coralSeen = false;
    coralYaw = 0;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    intake.setMotorVoltage(2);
    coralSeen = false;
  }

  @Override 
  public void execute() {
    if (vision.getValidCoral() == true) {
      coralSeen = true;
    }

    if (coralSeen == false) {
      drivetrain.applyRequest(() -> drive.withVelocityX(translationSup.getAsDouble() * MaxSpeed)
      .withVelocityY(strafeSupplier.getAsDouble() * MaxSpeed)
      .withRotationalRate(rotationSup.getAsDouble() * MaxAngularRate));
    } else {
      if (vision.getValidCoral() == true) {
        coralYaw = (vision.getCoralTX() * 2 / 3);
        rotation = Constants.SwerveCons.swerveRotationPID.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), 
          drivetrain.getState().Pose.getRotation().getDegrees() - coralYaw);
        strafe = Constants.SwerveCons.swerveStrafePID.calculate(coralYaw / 3, 0);
      }
      if (Math.abs(coralYaw) <= Constants.SwerveCons.maxIntakeError) {
        translation = 2.5;
      } else if (Math.abs(coralYaw) > Constants.SwerveCons.maxIntakeError) {
        translation = 1;
      }
      
      drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(translation, strafe, rotation)));
    }
  }

  @Override 
  public void end(boolean interrupted) {
      drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));
      intake.setMotorVoltage(0);

      coralSeen = false;
      coralYaw = 0;
  }

  @Override 
  public boolean isFinished() {
      if (intake.getLinebreakerEndEffectorStatus()) return true;
      else return false;
  }
}
