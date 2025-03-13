package frc.robot.Commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.libaries.LimelightHelpers;
import frc.robot.libaries.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;


public class UpdateVision extends Command {
  private CommandSwerveDrivetrain subDrivetrain;
  private VisionSubsystem poseEstimator;

  PoseEstimate estimatedPose;
  double drivetrainRotation = 0;
  
  public UpdateVision(CommandSwerveDrivetrain drivetrain, VisionSubsystem poseEstimator) {
    this.subDrivetrain = drivetrain;
    this.poseEstimator = poseEstimator;
    
    addRequirements(drivetrain, poseEstimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tells the limelight where we are on the field
    LimelightHelpers.SetRobotOrientation(VisionConstants.LIMELIGHT_NAMES[0],
        subDrivetrain.getRotation3d().getZ() * 57.2958, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(VisionConstants.LIMELIGHT_NAMES[1],
        subDrivetrain.getRotation3d().getZ() * 57.2958, 0, 0, 0, 0, 0);
    AngularVelocity gyroRate = subDrivetrain.getPigeon2().getAngularVelocityZWorld().getValue();

    Optional<PoseEstimate> estimatedPose = poseEstimator.determinePoseEstimate(gyroRate);
    if (estimatedPose.isPresent()) {
      subDrivetrain.addVisionMeasurement(estimatedPose.get().pose, estimatedPose.get().timestampSeconds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted == false) {
      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

 