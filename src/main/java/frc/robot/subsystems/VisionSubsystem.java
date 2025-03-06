// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libaries.LimelightHelpers;
import frc.robot.libaries.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
 private RawFiducial[] fiducials;

  //read values periodically

  public VisionSubsystem() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException { //No fiducial fonund
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VisionConstants.limelightName, new int[] {6,7,8,9,10,11,17,18,19,20,21,22});

    SmartDashboard.putNumber("Rotate P", 0.0);
    SmartDashboard.putNumber("Rotate D", 0.0);

  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials(Constants.VisionConstants.limelightName);
  }
  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;
    //Linear search for close
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }
    return closest;
  }
  
  //Linear searcgh by id
  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }


}
*/