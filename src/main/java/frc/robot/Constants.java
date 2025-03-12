// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;


import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;


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
            elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            //Adds upper and lower soft limits to the elevator 
            elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 52.7;
            elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.125;
            
            //Sets the feedforward gravity to that of an elevator not a arm. This removes the cosine function from the feedforward calculation
            elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

            //Rotations to Inches conversion
            elevatorConfig.Feedback.SensorToMechanismRatio = 1/6; 

            //Motion Magic® gains
            elevatorConfig.Slot0.kG = 0.4;
            elevatorConfig.Slot0.kS = 0.23;
            elevatorConfig.Slot0.kP = 2.0;
            elevatorConfig.Slot0.kI = 0.0;
            elevatorConfig.Slot0.kD = 0.02;
            elevatorConfig.Slot0.kV = 0.265;
            elevatorConfig.Slot0.kA = 0.002;
            elevatorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
           
            //Motion Magic® motion profile gains
            elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 52;
            elevatorConfig.MotionMagic.MotionMagicAcceleration = 120;
            elevatorConfig.MotionMagic.MotionMagicJerk = 800;
        }
        //Another set of motor configs for testing
        public static TalonFXConfiguration testConfigs = new TalonFXConfiguration();
        static {
            testConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        //Moter IDs
        public static final int elevatorLeftID = 13;
        public static final int elevatorRightID = 14;

        //Test Voltage for testing purposes
        public static final double testVoltage = 0.12;

        //Elevator pulley pitch diameter
        public static final double pulleyPitch = 7.0874330265;

    
        //Dis tances 
        public static final double L1 = 10;
        public static final double L2 = 17.2;
        public static final double L3 = 30.35;
        public static final double L4 = 52.55;
        public static final double home = -0.05;
    }

    public static class CANdleCons {
        public static final CANdleConfiguration LedConfig = new CANdleConfiguration();
        static {
            //Sets brightness from 0 to 1
            LedConfig.brightnessScalar = .5;
            LedConfig.stripType = LEDStripType.RGB;
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
            endEffectorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        //CAN ID for end effector Kraken
        public static final int endEffectorID = 15;

        //Digit port input for line breaker
        public static final int lineBreakerEndEffector = 1;
        public static final int lineBreakerTrough = 2;

        //Motor voltages
        public static final double intakeVoltage = 4.0;
        public static final double L1Voltage = 4.0;
        public static final double midLVoltage = 4.0;
        public static final double L4Voltage = 4.0;
    }

    public static class VisionConstants {
        public static final String[] LIMELIGHT_NAMES = new String[] { "limelight-right", "limelight-left" };

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * <p>
     * <b>Units:</b> Meters
     */
    public static final double MEGA_TAG2_STD_DEVS_POSITION = 0.7;

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final double MEGA_TAG2_STD_DEVS_HEADING = 9999999;

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Meters
     */
    public static final double MEGA_TAG1_STD_DEVS_POSITION = .3;

    public static final double MEGA_TAG1_STD_DEVS_HEADING = .1;
    /**
     * <p>
     * Maximum rate of rotation before we begin rejecting pose updates
     * </p>
     */
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(720);

    /**
     * The area that one tag (if its the only tag in the update) needs to exceed
     * before being accepted
     */
    public static final double AREA_THRESHOLD = 0.1;

    }

}
