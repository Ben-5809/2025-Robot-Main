// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.LEDsub;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Stripper;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private boolean isManual = false;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSub elevatorSub = new ElevatorSub();
    private final LEDsub ledSub = new LEDsub();
    private final CoralEndEffector coralEndEffector = new CoralEndEffector();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final Stripper stripper = new Stripper();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Align with Vision Left", new AutoAlign(visionSubsystem, drivetrain, true, Constants.VisionConstants.LIMELIGHT_NAMES[0]));
        NamedCommands.registerCommand("Align with Vision Right", new AutoAlign(visionSubsystem, drivetrain, false, Constants.VisionConstants.LIMELIGHT_NAMES[1]));
        NamedCommands.registerCommand("L1", new UnjamIntake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedBlue, -4));
        NamedCommands.registerCommand("L4", new  ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L4)
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3.25))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        NamedCommands.registerCommand("L3", new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L3)
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        NamedCommands.registerCommand("Intake", new Intake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 1.6));

        autoChooser = AutoBuilder.buildAutoChooser("Sigma");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();
    }

    private Command align() {
        return AutoBuilder.pathfindToPose(Constants.constField.POSES.REEF_A, Constants.VisionConstants.ampConstraints);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /* 
        operatorController.back().and(operatorController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        operatorController.back().and(operatorController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        operatorController.start().and(operatorController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        operatorController.start().and(operatorController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */        

        // reset the field-centric heading on left bumper press
        operatorController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Manual drive for the elevator 
        operatorController.y().whileTrue(new ElevatorApplyVoltage(elevatorSub, 2));
        operatorController.x().whileTrue(new ElevatorApplyVoltage(elevatorSub, -2));
        //operatorController.povUp().whileTrue();
        operatorController.povUp().whileTrue(align());

        //Trickshot
        operatorController.rightBumper().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.darkGreen, 19.8).andThen(new UnjamIntake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedPink, -2)));
        operatorController.b().onTrue(new UnjamIntake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, -3.3));
        operatorController.a().onTrue(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3.25));

        operatorController.leftTrigger(.9).whileTrue(new IntakeBall(coralEndEffector, stripper, Constants.CoralEndEffectorCons.intakeBall, Constants.StripperConstants.intakeVoltage));
        operatorController.rightTrigger(.9).whileTrue(new ScoreBarge(coralEndEffector, stripper, Constants.CoralEndEffectorCons.outtakeBall, Constants.StripperConstants.outtakeVoltage));

        operatorController.povRight().whileTrue(new AutoAlign(visionSubsystem, drivetrain, false, Constants.VisionConstants.LIMELIGHT_NAMES[1]));
        operatorController.povLeft().whileTrue(new AutoAlign(visionSubsystem, drivetrain, true, Constants.VisionConstants.LIMELIGHT_NAMES[0]));

        //driverController.y().onTrue(new UnjamIntake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, -4));
        driverController.a().onTrue(new Intake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 1.6));
        driverController.b().onTrue(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 0));
        driverController.x().onTrue(new UnjamIntake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 4));

        //Elevator controller commands
         
        driverController.leftTrigger(.9).onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L1)
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.rightTrigger(.9).onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L2)
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.leftBumper().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L3)
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.rightBumper().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L4)
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3.25))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.y().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home));
        
        driverController.povRight().whileTrue(new DriveToTag(visionSubsystem, drivetrain, false, Constants.VisionConstants.LIMELIGHT_NAMES[1]));
        driverController.povLeft().whileTrue(new DriveToTag(visionSubsystem, drivetrain, true, Constants.VisionConstants.LIMELIGHT_NAMES[0]));

        /* 
        //---------------------------------------------NEW CONTROLS--------------------------------------------------------------------
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        driverController.a().onTrue(new Intake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 1.6));
        driverController.b().onTrue(new UnjamIntake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, -3.3));
        driverController.x().onTrue(new UnjamIntake(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, -1.5));

        driverController.y().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home));

        driverController.rightBumper().onTrue(new AutoAlign(visionSubsystem, drivetrain, false, Constants.VisionConstants.LIMELIGHT_NAMES[1])
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.L4))
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3.25))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.leftBumper().onTrue(new AutoAlign(visionSubsystem, drivetrain, true, Constants.VisionConstants.LIMELIGHT_NAMES[0])
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.L4))
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3.25))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.rightTrigger(.9).onTrue(new AutoAlign(visionSubsystem, drivetrain, false, Constants.VisionConstants.LIMELIGHT_NAMES[1])
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.L3))
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 4))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.leftTrigger(.9).onTrue(new AutoAlign(visionSubsystem, drivetrain, true, Constants.VisionConstants.LIMELIGHT_NAMES[0])
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.L3))
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 4))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.povRight().onTrue(new AutoAlign(visionSubsystem, drivetrain, false, Constants.VisionConstants.LIMELIGHT_NAMES[1])
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.L2))
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 4))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));
        driverController.povLeft().onTrue(new AutoAlign(visionSubsystem, drivetrain, true, Constants.VisionConstants.LIMELIGHT_NAMES[0])
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.L2))
            .andThen(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 4))
            .andThen(new WaitCommand(0.16))
            .andThen(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.defualtColor, Constants.ElevatorCons.home)));


        //----------OPERATOR CONTROLS----------
        operatorController.leftTrigger(.9).onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedPink, Constants.ElevatorCons.L4));
        operatorController.rightTrigger(.9).whileTrue(new ScoreBarge(coralEndEffector, stripper, Constants.CoralEndEffectorCons.outtakeBall, Constants.StripperConstants.outtakeVoltage));
        operatorController.leftBumper().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.darkBlue, Constants.ElevatorCons.Algae1)
            .andThen(new IntakeBall(coralEndEffector, stripper, Constants.CoralEndEffectorCons.intakeBall, Constants.StripperConstants.intakeVoltage)));
        operatorController.rightBumper().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.darkBlue, Constants.ElevatorCons.Algae2)
            .andThen(new IntakeBall(coralEndEffector, stripper, Constants.CoralEndEffectorCons.intakeBall, Constants.StripperConstants.intakeVoltage)));
        
                
        //Manual drive for the elevator 
        operatorController.y().whileTrue(new ElevatorApplyVoltage(elevatorSub, 2));
        operatorController.x().whileTrue(new ElevatorApplyVoltage(elevatorSub, -2));
        operatorController.a().onTrue(new EndEffectorVoltage(coralEndEffector, ledSub, Constants.CANdleCons.saturatedGreen, 3.25));
        operatorController.b().onTrue(new ParallelCommandGroup(
            new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedPurple, Constants.ElevatorCons.home),
            new IntakeBall(coralEndEffector, stripper, 0, 0)));

        operatorController.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        */
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command UpdateVision () {
        return new UpdateVision(drivetrain, visionSubsystem)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).ignoringDisable(true);
    }

    public void setMegaTag2(boolean setMegaTag2) {

        if (setMegaTag2) {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(
            VisionConstants.MEGA_TAG2_STD_DEVS_POSITION,
            VisionConstants.MEGA_TAG2_STD_DEVS_POSITION,
            VisionConstants.MEGA_TAG2_STD_DEVS_HEADING));
        } else {
        // Use MegaTag 1
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(
            VisionConstants.MEGA_TAG1_STD_DEVS_POSITION,
            VisionConstants.MEGA_TAG1_STD_DEVS_POSITION,
            VisionConstants.MEGA_TAG1_STD_DEVS_HEADING));
        }
        visionSubsystem.setMegaTag2(setMegaTag2);
  }
}
