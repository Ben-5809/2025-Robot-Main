// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Stripper extends SubsystemBase {
  //Motor and sensor setup
  private TalonFX stripperMotor;

  public Stripper() {
    stripperMotor = new TalonFX(Constants.StripperConstants.stripperID);

    stripperMotor.getConfigurator().apply(Constants.StripperConstants.stripper);
  }

  //Applies a set voltage to the motors
  public void setMotorVoltage (double voltageApplied) {
    stripperMotor.setVoltage(voltageApplied);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
