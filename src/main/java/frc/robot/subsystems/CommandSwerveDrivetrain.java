package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.constField;;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    private double timeFromLastUpdate;
    private double lastSimTime;

    private SwerveModuleState lastDesiredSwerveModuleState;

    private SwerveModuleState[] lastDesiredStates = new SwerveModuleState[]{new SwerveModuleState(),
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    private PositionVoltage steerMotorController = new PositionVoltage(0);

    private DutyCycleOut driveMotorControllerOpen;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }



    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        timeFromLastUpdate = Timer.getFPGATimestamp() - lastSimTime;
		lastSimTime = Timer.getFPGATimestamp();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
	 * Minimize the change in heading the desired swerve module state would require
	 * by potentially reversing the direction the wheel spins. Customized from
	 * WPILib's version to include placing in appropriate scope for CTRE onboard
	 * control.
	 *
	 * @param desiredState
	 *            The desired state.
	 * @param currentAngle
	 *            The current module angle.
	 */
	private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
		double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
		double targetSpeed = desiredState.speedMetersPerSecond;
		double delta = targetAngle - currentAngle.getDegrees();
		if (Math.abs(delta) > 90) {
			targetSpeed = -targetSpeed;
			targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
		}
		return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
	}

    /**
	 * Takes the current angle and the target angle and outputs a new target angle
	 * that is within +/- 180 degrees of the current angle.
	 *
	 * The difference of the current angle of the module and the desired angle could
	 * be greater than 360 degrees, and thats a problem, and this function solves
	 * it.
	 *
	 * Note that this is simply helping the optimize function. This function only
	 * gets the angle within +/- 180 degrees, NOT +/- 90 degrees which is what the
	 * optimize function does.
	 *
	 *
	 * @param scopeReference
	 *            Current Angle in Degrees
	 * @param newAngle
	 *            Target Angle in Degrees
	 * @return Closest angle within scope in Degrees
	 */
	private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
		double lowerBound;
		double upperBound;
		double lowerOffset = scopeReference % 360;
		if (lowerOffset >= 0) {
			lowerBound = scopeReference - lowerOffset;
			upperBound = scopeReference + (360 - lowerOffset);
		} else {
			upperBound = scopeReference - lowerOffset;
			lowerBound = scopeReference - (360 + lowerOffset);
		}
		while (newAngle < lowerBound) {
			newAngle += 360;
		}
		while (newAngle > upperBound) {
			newAngle -= 360;
		}
		if (newAngle - scopeReference > 180) {
			newAngle -= 360;
		} else if (newAngle - scopeReference < -180) {
			newAngle += 360;
		}
		return newAngle;
	}

    private void setModuleState(SwerveModuleState desiredState, CommandSwerveDrivetrain drivetrain, boolean steerInPlace, int mod) {
		// Optimize explanation: https://youtu.be/0Xi9yb1IMyA?t=226
		SwerveModuleState state = drivetrain.optimize(desiredState, drivetrain.getModule(mod).getCurrentState().angle);
		lastDesiredSwerveModuleState = state;
		// -*- Setting the Drive Motor -*-


        // The output is from -1 to 1. Essentially a percentage
        // So, the requested speed divided by it's max speed.
        driveMotorControllerOpen.Output = (state.speedMetersPerSecond / 4.7);
        drivetrain.getModule(mod).getDriveMotor().setControl(driveMotorControllerOpen);

		// -*- Setting the Steer Motor -*-

		double rotation = state.angle.getRotations();

		// If the requested speed is lower than a relevant steering speed,
		// don't turn the motor. Set it to whatever it's previous angle was.
		if (Math.abs(state.speedMetersPerSecond) < (0.01 * 4.7) && !steerInPlace) {
			return;
		}

		steerMotorController.Position = rotation;
		drivetrain.getModule(mod).getSteerMotor().setControl(steerMotorController);
	}

    public void setModuleStates(SwerveModuleState[] desiredModuleStates, CommandSwerveDrivetrain drivetrain) {
		// Lowers the speeds if needed so that they are actually achievable. This has to
		// be done here because speeds must be lowered relative to the other speeds as
		// well
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates,4.7);
		lastDesiredStates = desiredModuleStates;

        int modNum = 0;

        SwerveModule[] modules;

        while (modNum != 3) {
            modules[modNum] = drivetrain.getModule(modNum);
            modNum += 1;
        }

        modNum = 0;
		for (SwerveModule mod : modules) {
			setModuleState(desiredModuleStates[modNum], drivetrain, false, modNum);
            modNum += 1;
		}
	}

    private void driveWithKinematics (ChassisSpeeds chassisSpeeds, CommandSwerveDrivetrain drivetrain) {
        SwerveDriveKinematics kinematics = drivetrain.getKinematics();
        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
        
    }


    private Pose2d getPose (CommandSwerveDrivetrain drivetrain) {
        return drivetrain.getPose(drivetrain);
    }

    /**
   * Returns the closest reef branch to the robot.
   * 
   * @param leftBranchRequested If we are requesting to align to the left or right
   *                            branch
   * @return The desired reef branch face to align to
   */
    public Pose2d getDesiredReef(boolean leftBranchRequested) {
    // Get the closest reef branch face using either branch on the face
    List<Pose2d> reefPoses = constField.getReefPositions().get();
    Pose2d currentPose = getState().Pose;
    Pose2d desiredReef = currentPose.nearest(reefPoses);
    int closestReefIndex = reefPoses.indexOf(desiredReef);

    // Invert faces on the back of the reef so they're always relative to the driver
    if (closestReefIndex > 3 && closestReefIndex < 10) {
      leftBranchRequested = !leftBranchRequested;
    }

    // If we were closer to the left branch but selected the right branch (or
    // vice-versa), switch to our desired branch
    if (leftBranchRequested && (closestReefIndex % 2 == 1)) {
      desiredReef = reefPoses.get(closestReefIndex - 1);
    } else if (!leftBranchRequested && (closestReefIndex % 2 == 0)) {
      desiredReef = reefPoses.get(closestReefIndex + 1);
    }
    return desiredReef;
  }

  /**
   * Calculate the ChassisSpeeds needed to align the robot to the desired pose.
   * This must be called every loop until you reach the desired pose.
   * 
   * @param desiredPose The desired pose to align to
   * @return The ChassisSpeeds needed to align the robot to the desired pose
   */
  public ChassisSpeeds getAlignmentSpeeds(Pose2d desiredPose, CommandSwerveDrivetrain subDrivetrain) {
    Pose2d desiredAlignmentPose = desiredPose;
    // TODO: This might run better if instead of 0, we use
    // constDrivetrain.TELEOP_AUTO_ALIGN.DESIRED_AUTO_ALIGN_SPEED.in(Units.MetersPerSecond);.
    // I dont know why. it might though
    return Constants.TELEOP_AUTO_ALIGN.TELEOP_AUTO_ALIGN_CONTROLLER.calculate(getPose(subDrivetrain), desiredPose, 0,
        desiredPose.getRotation());
  }

  /**
   * Contains logic for automatically aligning & automatically driving to the
   * reef.
   * May align only rotationally, automatically drive to a branch, or be
   * overridden by the driver
   */
  
  public void autoAlign(Distance distanceFromTarget, Pose2d desiredTarget,
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rVelocity, 
      CommandSwerveDrivetrain drivetrain) {
    Pose2d desiredAlignmentPose = desiredTarget;
    int redAllianceMultiplier = constField.isRedAlliance() ? -1 : 1;

    
    // Full auto-align
    ChassisSpeeds desiredChassisSpeeds = getAlignmentSpeeds(desiredTarget, drivetrain);

    // Speed limit based on elevator height
    LinearVelocity linearSpeedLimit = Units.MetersPerSecond.of(4.5);
    AngularVelocity angularSpeedLimit = Units.DegreesPerSecond.of(360);

    if (!RobotState.isAutonomous()) {
        if ((desiredChassisSpeeds.vxMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
            || (desiredChassisSpeeds.vyMetersPerSecond > linearSpeedLimit.in(Units.MetersPerSecond))
            || (desiredChassisSpeeds.omegaRadiansPerSecond > angularSpeedLimit.in(Units.RadiansPerSecond))) {

        desiredChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vxMetersPerSecond, 0,
            linearSpeedLimit.in(MetersPerSecond));
        desiredChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(desiredChassisSpeeds.vyMetersPerSecond, 0,
            linearSpeedLimit.in(MetersPerSecond));
        desiredChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(desiredChassisSpeeds.omegaRadiansPerSecond, 0,
            angularSpeedLimit.in(RadiansPerSecond));
        
    }

    drive(desiredChassisSpeeds);
    }
  }
}

