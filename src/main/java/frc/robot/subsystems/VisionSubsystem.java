// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.libaries.LimelightHelpers;
import frc.robot.libaries.LimelightHelpers.PoseEstimate;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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

  Pose2d desiredAlignmentPose = Pose2d.kZero;


  SwerveRequest.ApplyRobotSpeeds visionRequest = new SwerveRequest.ApplyRobotSpeeds();

  int[] invalidIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  
  public VisionSubsystem() {
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VisionConstants.LIMELIGHT_NAMES[0], invalidIDs);
    LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VisionConstants.LIMELIGHT_NAMES[1], invalidIDs);
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

  public double getTagIDRight () {
    return LimelightHelpers.getFiducialID(Constants.VisionConstants.LIMELIGHT_NAMES[0]);
  }

  public double getTagIDLeft () {
    return LimelightHelpers.getFiducialID(Constants.VisionConstants.LIMELIGHT_NAMES[1]);
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
  public Pose2d getTargetPos (boolean isLeft) {
    if (isLeft) {
      if (getTagIDRight() == 18 || getTagIDLeft() == 18) {
        return Constants.constField.POSES.REEF_A;
      } if (getTagIDRight() == 17 || getTagIDLeft() == 17) {
        return Constants.constField.POSES.REEF_C;
      } if (getTagIDRight() == 22 || getTagIDLeft() == 22) {
        return Constants.constField.POSES.REEF_E;
      } if (getTagIDRight() == 21 || getTagIDLeft() == 21) {
        return Constants.constField.POSES.REEF_G;
      } if (getTagIDRight() == 20 || getTagIDLeft() == 20) {
        return Constants.constField.POSES.REEF_I;
      } if (getTagIDRight() == 19 || getTagIDLeft() == 19) {
        return Constants.constField.POSES.REEF_K;
      } if (getTagIDRight() == 7 || getTagIDLeft() == 7) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_A);
      } if (getTagIDRight() == 8 || getTagIDLeft() == 8) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_C);
      } if (getTagIDRight() == 9 || getTagIDLeft() == 9) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_E);
      } if (getTagIDRight() == 10 || getTagIDLeft() == 10) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_G);
      } if (getTagIDRight() == 11 || getTagIDLeft() == 11) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_I);
      } if (getTagIDRight() == 6 || getTagIDLeft() == 6) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_K);
      } else  {
        System.out.println("No Tag Detected. No Target Pose Returned");
        return new Pose2d(0,0, new Rotation2d(0,0));
      }
    } else {
      if (getTagIDRight() == 18 || getTagIDLeft() == 18) {
        return Constants.constField.POSES.REEF_B;
      } if (getTagIDRight() == 17 || getTagIDLeft() == 17) {
        return Constants.constField.POSES.REEF_D;
      } if (getTagIDRight() == 22 || getTagIDLeft() == 22) {
        return Constants.constField.POSES.REEF_F;
      } if (getTagIDRight() == 21 || getTagIDLeft() == 21) {
        return Constants.constField.POSES.REEF_H;
      } if (getTagIDRight() == 20 || getTagIDLeft() == 20) {
        return Constants.constField.POSES.REEF_J;
      } if (getTagIDRight() == 19 || getTagIDLeft() == 19) {
        return Constants.constField.POSES.REEF_L;
      } if (getTagIDRight() == 7 || getTagIDLeft() == 7) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_B);
      } if (getTagIDRight() == 8 || getTagIDLeft() == 8) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_D);
      } if (getTagIDRight() == 9 || getTagIDLeft() == 9) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_F);
      } if (getTagIDRight() == 10 || getTagIDLeft() == 10) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_H);
      } if (getTagIDRight() == 11 || getTagIDLeft() == 11) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_J);
      } if (getTagIDRight() == 6 || getTagIDLeft() == 6) {
        return Constants.constField.getRedAlliancePose(Constants.constField.POSES.REEF_L);
      } else  {
        System.out.println("No Tag Detected. No Target Pose Returned");
        return new Pose2d(0,0, new Rotation2d(0,0));
      }
    }  
  }

  public ChassisSpeeds getAlignmentSpeeds(Pose2d desiredPose, CommandSwerveDrivetrain drivetrain) {
    desiredAlignmentPose = desiredPose;
  
    return Constants.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(drivetrain.getState().Pose, desiredPose, 0,
        desiredPose.getRotation());
  }

  public void autoAlignChassisSpeeds(Pose2d desiredTarget, CommandSwerveDrivetrain drivetrain) {
    desiredAlignmentPose = desiredTarget;

    ChassisSpeeds desiredChassisSpeeds = getAlignmentSpeeds(desiredTarget, drivetrain);

    desiredChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vxMetersPerSecond, 0,
      1);
    desiredChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vyMetersPerSecond, 0,
      1);
    desiredChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(desiredChassisSpeeds.omegaRadiansPerSecond, 0,
      1);
    
    drivetrain.setControl(visionRequest.withSpeeds(desiredChassisSpeeds));
  }

  public boolean isAtRotation(Rotation2d desiredRotation, CommandSwerveDrivetrain drivetrain) {
    return (drivetrain.getState().Pose.getRotation().getMeasure()
        .compareTo(desiredRotation.getMeasure().minus(Constants.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) > 0) &&
        drivetrain.getState().Pose.getRotation().getMeasure()
            .compareTo(desiredRotation.getMeasure().plus(Constants.TELEOP_AUTO_ALIGN.AT_ROTATION_TOLERANCE)) < 0;
  }

  public Boolean isAligned(CommandSwerveDrivetrain drivetrain) {
    return (desiredAlignmentPose.getTranslation().getDistance(
        drivetrain.getState().Pose.getTranslation()) <= Constants.TELEOP_AUTO_ALIGN.AUTO_ALIGNMENT_TOLERANCE.in(Units.Meters))
        && isAtRotation(desiredAlignmentPose.getRotation(), drivetrain);
  }
}