// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

public class ElevatorSub extends SubsystemBase {
  //Motors
  private TalonFX rightElevatorFollower;
  private TalonFX leftElevatorMaster;

  //Voltage var
  private VoltageOut voltageRequest;

  //Elevator distances
  private double currentLeftPos = 0;
  private double currentRightPos = 0;

  //Last position so I can signal log
  private double lastHeight = 0;

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
    rightElevatorFollower.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig.Slot0);
    leftElevatorMaster.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig.Slot0);
  }

  /**
   * Returns the distance of each motor in rotations. If they are basically the same, all is good in da hood
  */
  public double getElevatorPosRight () {
    return rightElevatorFollower.getPosition().getValueAsDouble();
  }
  public double getElevatorPosLeft () {
    return leftElevatorMaster.getPosition().getValueAsDouble();
  }

  public void setPositionZero () {
    leftElevatorMaster.setPosition(0);
    rightElevatorFollower.setPosition(0);
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
  public void setPosition (double height) {
    leftElevatorMaster.setControl(motionRequest.withPosition(height));
    rightElevatorFollower.setControl(new Follower(Constants.ElevatorCons.elevatorLeftID, true));
    lastHeight = height;
  }

  public double getLastHeight () {
    return lastHeight;
  }

  public void reZero () {
    rightElevatorFollower.setPosition(0);
    leftElevatorMaster.setPosition(0);
  }

  public void setSlotGains (int slot) {
    if (slot == 0) {
      rightElevatorFollower.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig.Slot0);
      leftElevatorMaster.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig.Slot0);
    } if (slot != 0) {
      rightElevatorFollower.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig.Slot1);
      leftElevatorMaster.getConfigurator().apply(Constants.ElevatorCons.elevatorConfig.Slot1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentRightPos = getElevatorPosRight();
    currentLeftPos = getElevatorPosLeft();
    SmartDashboard.putNumber("Left Elevator", currentLeftPos);
    SmartDashboard.putNumber("Reight Elevator", currentRightPos);

    SignalLogger.writeDouble("Motion Profile", leftElevatorMaster.getClosedLoopReference().getValueAsDouble());
    SignalLogger.writeDouble("Motion Magic Setpoint", lastHeight);
    SignalLogger.writeDouble("Current Elevator Pos", currentLeftPos);
  }
}
