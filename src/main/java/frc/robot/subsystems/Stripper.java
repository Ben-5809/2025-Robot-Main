// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Stripper extends SubsystemBase {
  //Motor and sensor setup
  private TalonSRX stripperMotor;

  public Stripper() {
    stripperMotor = new TalonSRX(Constants.StripperConstants.stripperID);
  }

  //Applies a set voltage to the motors
  public void setMotorVoltage (double voltageApplied) {
    stripperMotor.set(ControlMode.Current, voltageApplied);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
