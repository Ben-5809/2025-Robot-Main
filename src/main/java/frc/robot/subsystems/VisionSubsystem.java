// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.libaries.LimelightHelpers;
import frc.robot.libaries.LimelightHelpers.PoseEstimate;
import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

@Logged
public class VisionSubsystem extends SubsystemBase {
  PoseEstimate lastEstimateRight = new PoseEstimate();
  PoseEstimate lastEstimateLeft = new PoseEstimate();

  // Not logged, as they turn to false immediately after being read
  @NotLogged
  boolean newRightEstimate = false;
  @NotLogged
  boolean newLeftEstimate = false;

  Pose2d rightPose = new Pose2d();
  Pose2d leftPose = new Pose2d();

  private boolean useMegaTag2 = false;

  private Optional<Alliance> ALLIANCE = Optional.empty();

  int[] validIDs = {1-22};
  
  public VisionSubsystem() {
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VisionConstants.LIMELIGHT_NAMES[0], validIDs);
  }

  public PoseEstimate[] getLastPoseEstimates() {
    return new PoseEstimate[] { lastEstimateRight, lastEstimateLeft };
  }

  public void setMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  /**
   * Determines if a given pose estimate should be rejected.
   * 
   * 
   * @param poseEstimate The pose estimate to check
   * @param gyroRate     The current rate of rotation observed by our gyro.
   * 
   * @return True if the estimate should be rejected
   */

  public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate) {
    // Angular velocity is too high to have accurate vision
    if (gyroRate.compareTo(VisionConstants.MAX_ANGULAR_VELOCITY) > 0) {
      return true;
    }

    // No tags :<
    if (poseEstimate.tagCount == 0) {
      return true;
    }

    // 1 Tag with a large area
    if (poseEstimate.tagCount == 1 && poseEstimate.avgTagArea > VisionConstants.AREA_THRESHOLD) {
      return false;
      // 2 tags or more
    } else if (poseEstimate.tagCount > 1) {
      return false;
    }

    return true;
  }

  /**
   * Updates the current pose estimates for the left and right of the robot using
   * data from Limelight cameras.
   *
   * @param gyroRate The current angular velocity of the robot, used to validate
   *                 the pose estimates.
   *
   *                 This method retrieves pose estimates from two Limelight
   *                 cameras (left and right) and updates the
   *                 corresponding pose estimates if they are valid. The method
   *                 supports two modes of operation:
   *                 one using MegaTag2 and one without. The appropriate pose
   *                 estimate retrieval method is chosen
   *                 based on the value of the `useMegaTag2` flag.
   *
   *                 If the retrieved pose estimates are valid and not rejected
   *                 based on the current angular velocity,
   *                 the method updates the last known estimates and sets flags
   *                 indicating new estimates are available.
   */
  public void setCurrentEstimates(AngularVelocity gyroRate) {
    PoseEstimate currentEstimateRight = new PoseEstimate();
    PoseEstimate currentEstimateLeft = new PoseEstimate();

    if (useMegaTag2) {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAMES[0]);
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAMES[1]);
    } else {
      currentEstimateRight = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_NAMES[0]);
      currentEstimateLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_NAMES[1]);
    }

    if (currentEstimateRight != null && !rejectUpdate(currentEstimateRight, gyroRate)) {
      lastEstimateRight = currentEstimateRight;
      rightPose = currentEstimateRight.pose;
      newRightEstimate = true;
    }
    if (currentEstimateLeft != null && !rejectUpdate(currentEstimateLeft, gyroRate)) {
      lastEstimateLeft = currentEstimateLeft;
      leftPose = currentEstimateLeft.pose;
      newLeftEstimate = true;
    }
  }

  public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
    setCurrentEstimates(gyroRate);

    // No valid pose estimates :(
    if (!newRightEstimate && !newLeftEstimate) {
      return Optional.empty();

    } else if (newRightEstimate && !newLeftEstimate) {
      // One valid pose estimate (right)
      newRightEstimate = false;
      return Optional.of(lastEstimateRight);

    } else if (!newRightEstimate && newLeftEstimate) {
      // One valid pose estimate (left)
      newLeftEstimate = false;
      return Optional.of(lastEstimateLeft);

    } else {
      // Two valid pose estimates, disgard the one that's further
      newRightEstimate = false;
      newLeftEstimate = false;
      if (lastEstimateLeft.avgTagDist < lastEstimateRight.avgTagDist) {
        return Optional.of(lastEstimateRight);
      } else {
        return Optional.of(lastEstimateLeft);
      }
    }
  }

  @Override
  public void periodic() {
  }

  /**
   *  Calculates a ChassisSpeeds object for application to the drivetrain.
   * 
   *  @param targetPose a set point on the field that the robot is going to 
   *  @param robotPose determined by vision
   * 
   *  @return ChassisSpeeds for swerve drivetrain. 
   */
  public ChassisSpeeds calculateChassisSpeeds(Pose2d targetPose, Pose2d robotPose) {    
    //Adjustable factors for the speedcomponenets
    double speedFactor = 1;
    double rotationFactor = 1;

    //Gets the robot's current position and orientation
    Translation2d robotPosition = robotPose.getTranslation();
    Rotation2d robotRotation = robotPose.getRotation();
    
    //Gets the target's position
    Translation2d targetPosition = targetPose.getTranslation();

    //Calculates the difference in position (relative to the robot)
    Translation2d deltaPosition = targetPosition.minus(robotPosition);

    //Calculates the angle to the target
    double targetAngle = Math.atan2(deltaPosition.getY(), deltaPosition.getX());
    
    //Calculates the robot's angular difference to the target angle
    double deltaAngle = targetAngle - robotRotation.getRadians();
    
    //Normalizes the angle difference to between -π and π
    deltaAngle = Math.atan2(Math.sin(deltaAngle), Math.cos(deltaAngle));

    //Calculates the x velocity
    double vx = deltaPosition.getX() * speedFactor;
    if (vx < 0.02) {
      vx = 0; //If the motor is stalling set it zero 
    }

    //Calculates the y velocity
    double vy = deltaPosition.getY() * speedFactor;
    if (vx < 0.02) {
      vx = 0; //If the motor is stalling set it zero 
    }

    //Angular speed is proportional to the angular difference
    double angularSpeed = deltaAngle * rotationFactor;
    if (angularSpeed < 0.02) {
      angularSpeed = 0; //If the motor is stalling set it zero 
    }

    // Create and return the ChassisSpeeds object (vx, vy, omega)
    return new ChassisSpeeds(vx, vy, angularSpeed);
  }

  public double getTagID (String LimelightName) {
    return LimelightHelpers.getFiducialID(LimelightName);
  }

  /**
   *  Logic to figure out what position to target.
   *  It has three inputs: the alliance, which tag is seen, and if it is targeting the left or right reef
   * 
   *  @param LimelightName is the name of the limelight used to lineup
   *  @param isLeft {@code true} if lining up left, {@code false} for right
   *  
   *  @return Pose2d of target
   */
  public Pose2d getTargetPos (String LimelightName, boolean isLeft) {
    if (ALLIANCE.isPresent() && ALLIANCE.get() == Alliance.Blue) {
      if (isLeft) {
        if (getTagID(LimelightName) == 18) {
          return Constants.constField.POSES.REEF_A;
        } if (getTagID(LimelightName) == 17) {
          return Constants.constField.POSES.REEF_C;
        } if (getTagID(LimelightName) == 22) {
          return Constants.constField.POSES.REEF_E;
        } if (getTagID(LimelightName) == 21) {
          return Constants.constField.POSES.REEF_G;
        } if (getTagID(LimelightName) == 20) {
          return Constants.constField.POSES.REEF_I;
        } if (getTagID(LimelightName) == 19) {
          return Constants.constField.POSES.REEF_K;
        } else  {
          System.out.println("No Tag Detected. No Target Pose Returned");
          return null;
        }
      } else {
        if (getTagID(LimelightName) == 18) {
          return Constants.constField.POSES.REEF_B;
        } if (getTagID(LimelightName) == 17) {
          return Constants.constField.POSES.REEF_D;
        } if (getTagID(LimelightName) == 22) {
          return Constants.constField.POSES.REEF_F;
        } if (getTagID(LimelightName) == 21) {
          return Constants.constField.POSES.REEF_H;
        } if (getTagID(LimelightName) == 20) {
          return Constants.constField.POSES.REEF_J;
        } if (getTagID(LimelightName) == 19) {
          return Constants.constField.POSES.REEF_L;
        } else  {
          System.out.println("No Tag Detected. No Target Pose Returned");
          return null;
        }
      }
    } else {
      if (isLeft) {
        if (getTagID(LimelightName) == 18) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_A);
        } if (getTagID(LimelightName) == 17) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_C);
        } if (getTagID(LimelightName) == 22) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_E);
        } if (getTagID(LimelightName) == 21) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_G);
        } if (getTagID(LimelightName) == 20) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_I);
        } if (getTagID(LimelightName) == 19) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_K);
        } else  {
          System.out.println("No Tag Detected. No Target Pose Returned");
          return null;
        }
      } else {
        if (getTagID(LimelightName) == 18) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_B);
        } if (getTagID(LimelightName) == 17) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_D);
        } if (getTagID(LimelightName) == 22) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_F);
        } if (getTagID(LimelightName) == 21) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_H);
        } if (getTagID(LimelightName) == 20) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_J);
        } if (getTagID(LimelightName) == 19) {
          return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_L);
        } else  {
          System.out.println("No Tag Detected. No Target Pose Returned");
          return null;
        }
      }
    }
  }
}