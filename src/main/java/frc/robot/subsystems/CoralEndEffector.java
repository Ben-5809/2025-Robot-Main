// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralEndEffector extends SubsystemBase {
  //Motor and sensor setup
  private TalonFX endEffectorMotor;
  private DigitalInput lineBreakerEndEffector;
  private DigitalInput lineBreakerTrough;

  //Voltage var
  private VoltageOut voltageRequest;

  public CoralEndEffector() {
    endEffectorMotor = new TalonFX(Constants.CoralEndEffectorCons.endEffectorID);
    lineBreakerEndEffector = new DigitalInput(Constants.CoralEndEffectorCons.lineBreakerEndEffector);
    lineBreakerTrough = new DigitalInput(Constants.CoralEndEffectorCons.lineBreakerTrough);

    //Applying configs to kraken
    endEffectorMotor.getConfigurator().apply(Constants.CoralEndEffectorCons.endEffectorConfig);

    //Creating voltage request
    voltageRequest = new VoltageOut(0);

    //Sets internal encoder to 0
    endEffectorMotor.setPosition(0);
  }

  //Method to return encoder value
  public double getEncoderPosition () {
    return endEffectorMotor.getPosition().getValueAsDouble();
  }

  //Method to return status of linebreaker
  public boolean getLinebreakerEndEffectorStatus () {
    return lineBreakerEndEffector.get();
  }

  public boolean getLinebreakerTroughStatus () {
    return lineBreakerTrough.get();
  }

  //Applies a set voltage to the motors
  public void setMotorVoltage (double voltageApplied) {
    endEffectorMotor.setControl(voltageRequest.withOutput(voltageApplied));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
