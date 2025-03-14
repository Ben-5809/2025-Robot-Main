// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.libaries.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveToTag extends Command {

  public DriveToTag() {

  }

  //Overload for specific april tag by id

  //maybe set tagid by closest here instead of each execution to keep same tag
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    
    }
  

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}
