// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.LEDsub;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Subs
    private final ElevatorSub elevatorSub = new ElevatorSub();
    private final LEDsub ledSub = new LEDsub();

    public RobotContainer() {
        configureBindings();
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

        /* 
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
         
        operatorController.back().and(operatorController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        operatorController.back().and(operatorController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        operatorController.start().and(operatorController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        operatorController.start().and(operatorController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        

        // reset the field-centric heading on left bumper press
        operatorController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Manual drive for the elevator 
        driverController.y().whileTrue(new ElevatorApplyVoltage(elevatorSub, 3));
        driverController.x().whileTrue(new ElevatorApplyVoltage(elevatorSub, -3));

        //Elevator controller commands
        driverController.leftTrigger(.8).onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L1));
        driverController.rightTrigger(.8).onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L2));
        driverController.leftBumper().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L3));
        driverController.rightBumper().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.L4));
        operatorController.a().onTrue(new ElevatorController(elevatorSub, ledSub, Constants.CANdleCons.saturatedGreen, Constants.ElevatorCons.home));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
