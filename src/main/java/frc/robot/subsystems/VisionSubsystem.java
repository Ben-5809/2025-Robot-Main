// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libaries.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  double x;
  double y;
  double area;

  //read values periodically

  public VisionSubsystem() {}

  public double getTX () {
    return x = LimelightHelpers.getTX("limelight-left");
  }

  public double getTY () {
    return y = LimelightHelpers.getTY("limelight-left");
  }

  public double getTA () {
    return area = LimelightHelpers.getTA("limelight-left");
  }
  
  @Override
  public void periodic() {
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}
