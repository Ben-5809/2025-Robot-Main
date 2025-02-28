// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle; 

public class LEDsub extends SubsystemBase {
  CANdle LED;

  public LEDsub() {
    //Attaches ID to CANdle object and applies brightness configs
    LED = new CANdle(Constants.CANdleCons.CANdleID);
    LED.configAllSettings(Constants.CANdleCons.LedConfig);
  }

  public void setLED(int[] rgbValues) {
    LED.setLEDs(rgbValues[0], rgbValues[1], rgbValues[2]);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
