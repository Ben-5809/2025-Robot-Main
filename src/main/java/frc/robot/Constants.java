// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix.led.CANdleConfiguration;


//Constants Class for quick change to vars
public final class Constants {
    //Elavtor Subsystem Contstants and Config objects
    public static class ElevatorCons {
        //Motor config object for elevator drive motors. Will be applied to both
        public static TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        static {
            //When no opperation is running, motors will default to brake mode
            elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            
            //Sets the motors to the correct orientation
            elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            //Adds upper and lower soft limits to the elevator 
            elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
            elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -53.6;
            
            //Sets the feedforward gravity to that of an elevator not a arm. This removes the cosine function from the feedforward calculation
            elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

            //Rotations to Inches conversion
            elevatorConfig.Feedback.SensorToMechanismRatio = 1/6; 

            //Motion Magic® gains
            elevatorConfig.Slot0.kG = 0;
            elevatorConfig.Slot0.kS = 0;
            elevatorConfig.Slot0.kP = 0;
            elevatorConfig.Slot0.kI = 0;
            elevatorConfig.Slot0.kD = 0;
            elevatorConfig.Slot0.kV = 1;
            elevatorConfig.Slot0.kA = 0.25;
            elevatorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

            //Motion Magic® motion profile gains
            elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 40;
            elevatorConfig.MotionMagic.MotionMagicAcceleration = 80;
            elevatorConfig.MotionMagic.MotionMagicJerk = 800;
        }
        //Another set of motor configs for testing
        public static TalonFXConfiguration testConfigs = new TalonFXConfiguration();
        static {
            testConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        //Moter IDs
        public static final int elevatorLeftID = 13;
        public static final int elevatorRightID = 14;

        //Test Voltage for testing purposes
        public static final double testVoltage = 0.12;

        //Elevator pulley pitch diameter
        public static final double pulleyPitch = 7.0874330265;

        //Distances for reef heights
        public static final double L1 = -10;
        public static final double L2 = -22;
        public static final double L3 = -39;
        public static final double L4 = -53.6;
        public static final double home = 0;
    }

    public static class CANdleCons {
        public static final CANdleConfiguration LedConfig = new CANdleConfiguration();
        static {
            //Sets brightness from 0 to 1
            LedConfig.brightnessScalar = 1;
        }

        //CANdle CAN ID
        public static final int CANdleID = 21;

        //RGB values for LEDs
        public static final int[] saturatedGreen = { 0, 255, 0 }; 
        public static final int[] saturatedRed = { 255, 0, 0 }; 
        public static final int[] saturatedBlue = { 0, 0, 255 }; 
        public static final int[] saturatedCyan = { 23, 247, 255 };
        public static final int[] saturatedPink = { 255, 0, 225 }; 
        public static final int[] saturatedOrange = { 255, 153, 0 }; 
        public static final int[] saturatedYellow = { 255, 251, 0 }; 
        public static final int[] saturatedPurple = { 242, 0, 255 };
        public static final int[] darkGreen = { 8, 161, 0 }; 
        public static final int[] darkRed = { 161, 0, 0 }; 
        public static final int[] darkBlue = { 19, 0, 161 }; 

        public static final int[] defualtColor = { 23, 247, 255 };
    }

    public static class CoralEndEffectorCons {
        public static final TalonFXConfiguration endEffectorConfig = new TalonFXConfiguration();
        static {
            //Sets the neutral mode of endeffector motor to brake. This is so coral does not slip out
            endEffectorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            //Sets it to inverted. This is because of the belting 
            endEffectorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        //CAN ID for end effector Kraken
        public static final int endEffectorID = 15;

        //Digit port input for line breaker
        public static final int lineBreaker = 1;

        //Motor voltages
        public static final double intakeVoltage = 4.0;
        public static final double L1Voltage = 4.0;
        public static final double midLVoltage = 4.0;
        public static final double L4Voltage = 4.0;
    }
}
