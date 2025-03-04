// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.*;
import com.pathplanner.*;
import com.pathplanner.lib.config.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
            elevatorConfig.Slot0.kG = 0.19;
            elevatorConfig.Slot0.kS = 0.23;
            elevatorConfig.Slot0.kP = 2.0;
            elevatorConfig.Slot0.kI = 0.0;
            elevatorConfig.Slot0.kD = 0.02;
            elevatorConfig.Slot0.kV = 0.149;
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
        public static final double L3 = 30.3;
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

    public static class poseEstimatorCons {
        public static final String leftCamera = "limelight-left";
        public static final String rightCamera = "limelight-right";

        public static final double autoVisionStdDevs = 10;
        public static final double teleopVisionStdDevs = 0.01;
        public static final double aimVisionStdDevs = 0.01;
    }

    public static class VisionConstants {
        public static final String limelightName = "limelight-left";

        
        public static final double MOVE_P = 0.300000;
        public static final double MOVE_I = 0.000000;
        public static final double MOVE_D = 0.000600;

        public static final double ROTATE_P = 0.030000;
        public static final double ROTATE_I = 0.000000;
        public static final double ROTATE_D = 0.000100;

        public static final double TOLERANCE = 0.01;

        //Tag ID 11
        public static final Pose2d redReefStartPose = new Pose2d(
            new Translation2d(12.425, 2.925),
            new Rotation2d()
        );

        //Tag ID 20
        public static final Pose2d blueReefStartPose = new Pose2d(
            new Translation2d(5.125, 5.125),
            new Rotation2d()
        );
    }

    public static final class PathPlanner {
        public static final HolonomicPathFollowerConfig pathPlannerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants( // Translation PID constants
                5.0, 
                0.0, 
                0.0), 
            new PIDConstants( // Rotation PID constants
                5.0, 
                0.0, 
                0.0
            ),
            4.5,
            .41,
            new ReplanningConfig()
        );
    }


    public static final class Swerve {

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(22.75); //TODO: This must be tuned to specific robot

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    }
}
