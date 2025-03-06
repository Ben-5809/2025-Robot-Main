/* 
package frc.robot.subsystems;

import frc.robot.libaries.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PoseEstimatorSub extends SubsystemBase {
    public Pigeon2 gyro;

    public SwerveDrivePoseEstimator poseEstimator;

    CommandSwerveDrivetrain swerveSub;

    double visionStdDevs;

    Field2d field = new Field2d();

    private int tagCount;
    
    public PoseEstimatorSub() {
        //idk if this is the right id 
        gyro = new Pigeon2(51);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        visionStdDevs = Constants.poseEstimatorCons.autoVisionStdDevs;

        tagCount = 0;
    }

    public void initialize(CommandSwerveDrivetrain swerveSub) {
        this.swerveSub = swerveSub;

      
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(1),
            swerveSub.getState().ModulePositions,
            getStartingReefPose()
        );
    }

    public Rotation2d getGyroYaw(int zeroTemp) {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble() * zeroTemp);
        //fromDegrees(gyro.getYaw().getValue());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(1), swerveSub.getState().ModulePositions, pose);
    }

    public void setPoseTranslation(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(1), swerveSub.getState().ModulePositions, new Pose2d(pose.getTranslation(), getPose().getRotation()));
    }

    public void setPoseTranslation(Translation2d translation) {
        poseEstimator.resetPosition(getGyroYaw(1), swerveSub.getState().ModulePositions, new Pose2d(translation, getPose().getRotation()));
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Pose2d getStartingReefPose() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
          return Constants.VisionConstants.redReefStartPose;
      } else {
          return Constants.VisionConstants.blueReefStartPose;
      }
  }

    public Rotation2d getHeadingFieldOriented() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() == false) return getPose().getRotation();
        if (alliance.get() == DriverStation.Alliance.Blue) {
            return getPose().getRotation();
        } else {
            if (getPose().getRotation().getDegrees() <= 0) {
                return new Rotation2d(getPose().getRotation().getRadians() + Math.PI);
            } else return new Rotation2d(getPose().getRotation().getRadians() - Math.PI);
        }
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(1), swerveSub.getState().ModulePositions, new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() == true && alliance.get() == DriverStation.Alliance.Red) {
            poseEstimator.resetPosition(getGyroYaw(180), swerveSub.getState().ModulePositions, new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI)));
        } else {
            poseEstimator.resetPosition(getGyroYaw(180), swerveSub.getState().ModulePositions, new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }
    }

    public double getVisionStdDevs() {
        return visionStdDevs;
    }

    public void setVisionStdDevs(double visionStdDevs) {
        this.visionStdDevs = visionStdDevs;
    }

    public void setStandardVisionStdDevs() {
        if (DriverStation.isAutonomousEnabled() == true) {
            setVisionStdDevs(Constants.poseEstimatorCons.autoVisionStdDevs);
        } else {
            setVisionStdDevs(Constants.poseEstimatorCons.teleopVisionStdDevs);
        }
    }

    public void setTagCount(int tagCount) {
        this.tagCount = tagCount;
    }

    public int getTagCount() {
        return tagCount;
    }

    public Pose2d getReefPose() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
          return Constants.VisionConstants.redReefStartPose;
      } else {
          return Constants.VisionConstants.blueReefStartPose;
      }
  }

    public void update() {
        poseEstimator.update(getGyroYaw(1), swerveSub.getState().ModulePositions);
        
        boolean rejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.limelightName, 
            getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.limelightName);
        setTagCount(botPose.tagCount);
        if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) rejectUpdate = true;
        if (botPose.tagCount == 0) rejectUpdate = true;
        if (rejectUpdate == false) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(getVisionStdDevs(), getVisionStdDevs(), 9999999));
            poseEstimator.addVisionMeasurement(botPose.pose, botPose.timestampSeconds);
        }
        
        LimelightHelpers.PoseEstimate limelightBotpose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if(limelightBotpose.tagCount >= 1) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(getVisionStdDevs(), getVisionStdDevs(),9999999));
            poseEstimator.addVisionMeasurement(
                limelightBotpose.pose,
                Timer.getFPGATimestamp() -
                    (LimelightHelpers.getLatency_Pipeline("limelight-left") / 1000) -
                    (LimelightHelpers.getLatency_Capture("limelight-left") / 1000)
            );
                //limelightBotpose.timestampSeconds);
        }




        field.setRobotPose(getPose());
    }

    @Override 
    public void periodic() {

        update();
        
        SmartDashboard.putData("Field", field);
    }

    @Override 
    public void simulationPeriodic() {}
}

*/