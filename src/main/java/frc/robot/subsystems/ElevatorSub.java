// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

public class ElevatorSub extends SubsystemBase {
  //Motors
  private TalonFX rightElevatorFollower;
  private TalonFX leftElevatorMaster;

  //Voltage var
  VoltageOut voltageRequest;

  //Elevator distances
  Distance currentLeftPos = Units.Inches.of(0);
  Distance currentRightPos = Units.Inches.of(0);

  //Position request for motion magic. First calculates voltage from PID. Second calculate motion magic profile from pos endpoint
  PositionVoltage positionRequest;
  MotionMagicVoltage motionRequest;

  /** Creates a new ElevatorSub. */
  public ElevatorSub() {
    rightElevatorFollower = new TalonFX(Constants.ElevatorCons.elevatorRightID);
    leftElevatorMaster = new TalonFX(Constants.ElevatorCons.elevatorLeftID);

    //Set internal encoder to 0
    rightElevatorFollower.setPosition(0);
    leftElevatorMaster.setPosition(0);

    //VoltageOut object for the test voltage function
    voltageRequest = new VoltageOut(0);

    //Voltage position 
    positionRequest = new PositionVoltage(0);
    motionRequest = new MotionMagicVoltage(0);

    //Applies Talon Configs to the motors
    rightElevatorFollower.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig);
    leftElevatorMaster.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig);
  }

  //Returns the distance of each motor in inches. If they are basically the same, all is good in da hood
  public Distance getElevatorPosRight () {
    return Units.Inches.of(rightElevatorFollower.getPosition().getValueAsDouble());
  }
  public Distance getElevatorPosLeft () {
    return Units.Inches.of(leftElevatorMaster.getPosition().getValueAsDouble());
  }

  //Applies a small amount of voltage to the motors for testing purposes
  public void applyVoltage (double voltageApplied) {
    leftElevatorMaster.setControl(voltageRequest.withOutput(voltageApplied));
    rightElevatorFollower.setControl(new Follower(Constants.ElevatorCons.elevatorLeftID, true));
  }

  //Sets the motors to coast so mechanism can be moved by hand
  public void setTestMode(boolean testMode) {
    if (testMode) {
      leftElevatorMaster.getConfigurator().apply(Constants.ElevatorCons.testConfigs);
      rightElevatorFollower.getConfigurator().apply(Constants.ElevatorCons.testConfigs);
    }
    else {
      rightElevatorFollower.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig);
      leftElevatorMaster.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig);
    }
  }

  //The only important function really. Creates a new setpoint for the motion profile
  public void setPosition (Distance height) {
    leftElevatorMaster.setControl(motionRequest.withPosition(height.in(Units.Inches)));
    rightElevatorFollower.setControl(new Follower(Constants.ElevatorCons.elevatorLeftID, true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentRightPos = getElevatorPosRight();
    currentLeftPos = getElevatorPosLeft();
    SmartDashboard.putData("Right Elevator", (Sendable) currentRightPos);
    SmartDashboard.putData("Left Elevator", (Sendable) currentLeftPos);
  }
}
