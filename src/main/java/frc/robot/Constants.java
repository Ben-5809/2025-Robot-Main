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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.constField.POSES;

import com.ctre.phoenix.led.CANdleConfiguration;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

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

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();
    public static final Distance FIELD_LENGTH = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
    public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(5));

    /**
     * Boolean that controls when the path will be mirrored for the red
     * alliance. This will flip the path being followed to the red side of the
     * field.
     * The origin will remain on the Blue side.
     * 
     * @return If we are currently on Red alliance. Will return false if no alliance
     *         is found
     */
    public static boolean isRedAlliance() {
      var alliance = ALLIANCE;
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    /*
     * All poses on the field, defined by their location on the BLUE Alliance
     */
    public static class POSES {
      public static final Pose2d RESET_POSE = new Pose2d(3.169, 4.015, new Rotation2d());

      public static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

      // BRANCH POSES
      public static final Pose2d REEF_A = new Pose2d(3.171, 4.189, Rotation2d.fromDegrees(0));
      public static final Pose2d REEF_B = new Pose2d(3.171, 3.863, Rotation2d.fromDegrees(0));
      public static final Pose2d REEF_C = new Pose2d(3.688, 2.968, Rotation2d.fromDegrees(60));
      public static final Pose2d REEF_D = new Pose2d(3.975, 2.803, Rotation2d.fromDegrees(60));
      public static final Pose2d REEF_E = new Pose2d(5.001, 2.804, Rotation2d.fromDegrees(120));
      public static final Pose2d REEF_F = new Pose2d(5.285, 2.964, Rotation2d.fromDegrees(120));
      public static final Pose2d REEF_G = new Pose2d(5.805, 3.863, Rotation2d.fromDegrees(180));
      public static final Pose2d REEF_H = new Pose2d(5.805, 4.189, Rotation2d.fromDegrees(180));
      public static final Pose2d REEF_I = new Pose2d(5.288, 5.083, Rotation2d.fromDegrees(-120));
      public static final Pose2d REEF_J = new Pose2d(5.002, 5.248, Rotation2d.fromDegrees(-120));
      public static final Pose2d REEF_K = new Pose2d(3.972, 5.247, Rotation2d.fromDegrees(-60));
      public static final Pose2d REEF_L = new Pose2d(3.693, 5.079, Rotation2d.fromDegrees(-60));

      private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
      private static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();

      // CORAL STATION POSES
      public static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(-54.5));
      public static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(-54.5));
      public static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(55));
      public static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(55));

      private static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
          LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
      private static final List<Pose2d> RED_CORAL_STATION_POSES = getRedCoralStationPoses();

      // processor poses
      public static final Pose2d PROCESSOR = new Pose2d(6, .77, Rotation2d.fromDegrees(-90));

      private static final Pose2d BLUE_PROCESSOR_POSE = PROCESSOR;
      private static final Pose2d RED_PROCESSOR_POSE = getRedProcessorPose();
      private static final List<Pose2d> PROCESSOR_POSES = List.of(BLUE_PROCESSOR_POSE, RED_PROCESSOR_POSE);

      private static final Pose2d[] BLUE_POSES = new Pose2d[] { RESET_POSE, REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L };

      private static final Pose2d[] RED_POSES = getRedAlliancePoses();

    }

    public static Pose2d getRedAlliancePose(Pose2d bluePose) {
      return new Pose2d(FIELD_LENGTH.in(Units.Meters) - (bluePose.getX()),
          FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    private static Pose2d[] getRedAlliancePoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_POSES.length];

      for (int i = 0; i < POSES.BLUE_POSES.length; i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_POSES[i]);
      }
      return returnedPoses;
    }

    private static List<Pose2d> getRedReefPoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_REEF_POSES.size()];

      for (int i = 0; i < POSES.BLUE_REEF_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_REEF_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9], returnedPoses[10],
          returnedPoses[11]);
    }

    private static List<Pose2d> getRedCoralStationPoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_CORAL_STATION_POSES.size()];

      for (int i = 0; i < POSES.BLUE_CORAL_STATION_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_CORAL_STATION_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3]);
    }

    private static Pose2d getRedProcessorPose() {
      Pose2d returnedPose = POSES.BLUE_PROCESSOR_POSE;

      returnedPose = getRedAlliancePose(POSES.BLUE_PROCESSOR_POSE);

      return returnedPose;
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of field element positions
     */
    public static Supplier<Pose2d[]> getFieldPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_POSES;

      }
      return () -> POSES.BLUE_POSES;
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of the reef branches for your alliance
     */
    public static Supplier<List<Pose2d>> getReefPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_REEF_POSES;

      }
      return () -> POSES.BLUE_REEF_POSES;
    }

    public static Supplier<List<Pose2d>> getCoralStationPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_CORAL_STATION_POSES;
      }
      return () -> POSES.BLUE_CORAL_STATION_POSES;
    }

    public static Supplier<List<Pose2d>> getProcessorPositions() {
      return () -> POSES.PROCESSOR_POSES;
    }

  }
}