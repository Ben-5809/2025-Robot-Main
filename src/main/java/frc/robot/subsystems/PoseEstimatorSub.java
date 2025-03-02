// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorSub extends SubsystemBase {

  public Pigeon2 gyro;

  public SwerveDrivePoseEstimator poseEstimator;

  Field2d field = new Field2d();

  public PoseEstimatorSub() {
    gyro = new Pigeon2(51);
    gyro.setYaw(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
