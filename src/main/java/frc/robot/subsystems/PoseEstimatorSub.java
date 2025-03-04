package frc.robot.subsystems;

import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
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

    private boolean autoNoteSeen;

    private int tagCount;
    
    public PoseEstimatorSub() {
        //idk if this is the right id 
        gyro = new Pigeon2(51);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        visionStdDevs = Constants.poseEstimatorCons.autoVisionStdDevs;

        autoNoteSeen = false;

        tagCount = 0;
    }

    public void initialize(CommandSwerveDrivetrain swerveSub) {
        this.swerveSub = swerveSub;

      
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            swerveSub.getState().ModulePositions,
            getCloseSpeakerPose()
        );
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getState().ModulePositions,, pose);
    }

    public void setPoseTranslation(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getState().ModulePositions,, new Pose2d(pose.getTranslation(), getPose().getRotation()));
    }

    public void setPoseTranslation(Translation2d translation) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getState().ModulePositions,, new Pose2d(translation, getPose().getRotation()));
    }

    public void resetPoseToCloseSpeaker() {
        setPoseTranslation(getCloseSpeakerPose());
    }
    
    public Rotation2d getHeading() {
        return getPose().getRotation();
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
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getState().ModulePositions, new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() == true && alliance.get() == DriverStation.Alliance.Red) {
            poseEstimator.resetPosition(getGyroYaw(), swerveSub.getState().ModulePositions, new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI)));
        } else {
            poseEstimator.resetPosition(getGyroYaw(), swerveSub.getState().ModulePositions, new Pose2d(getPose().getTranslation(), new Rotation2d()));
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

    public Pose2d getCloseSpeakerPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.poseEstimatorCons.redCloseSpeakerPose;
            } else {
                return Constants.poseEstimatorCons.blueCloseSpeakerPose;
            }
        }
        else return Constants.poseEstimatorCons.blueCloseSpeakerPose;
    }

    public Pose2d getSpeakerPoseYaw() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.poseEstimatorCons.redSpeakerPoseYaw;
            } else {
                return Constants.poseEstimatorCons.blueSpeakerPoseYaw;
            }
        }
        else return Constants.poseEstimatorCons.blueSpeakerPoseYaw;
    }

    public Pose2d getSpeakerPoseDistance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.poseEstimatorCons.redSpeakerPoseDistance;
            } else {
                return Constants.poseEstimatorCons.blueSpeakerPoseDistance;
            }
        }
        else return Constants.poseEstimatorCons.blueSpeakerPoseYaw;
    }

    public boolean getValidSpeaker() {
        if (LimelightHelpers.getTV("limelight-Shooter") == true) return true;
        else return false;
    }

    public double getSpeakerTX() {
        return LimelightHelpers.getTX("limelight-shooter");
    }

    public double getSpeakerTY() {
        return LimelightHelpers.getTY("limelight-shooter");
    }

    public double getCameraSpeakerDistance() {
        return (
            (Constants.poseEstimatorCons.speakerTagHeight - Constants.poseEstimatorCons.cameraHeight) /
            Math.tan(Math.PI / 180 * (
                30 + getSpeakerTY()
            ))
        );
    }
  
    public double getSpeakerYaw() {
        double yaw = (getPose().getRotation().getDegrees() - 90 + (180 / Math.PI * Math.acos(
            (
                Math.pow(getCameraSpeakerDistance(), 2) - 
                Math.pow(getSpeakerDistance(), 2) -
                Math.pow(Constants.poseEstimatorCons.cameraOffset, 2)
            ) / (
                -2 * getCameraSpeakerDistance() * getSpeakerDistance()
            )
        )));

        yaw = getPose().getRotation().getDegrees() - LimelightHelpers.getTX("limelight-shooter");

        if (yaw > 180) return yaw - 360;
        else if (yaw <= -180) return yaw + 360;
        else return yaw;
    }  
    
    public double getSpeakerDistance() {
        return Math.sqrt(
            Math.pow(getPose().getX() - getSpeakerPoseDistance().getX(), 2) +
            Math.pow(getPose().getY() - getSpeakerPoseDistance().getY(), 2) + .2
        );
    }

    public void update() {
        poseEstimator.update(getGyroYaw(), swerveSub.getState().ModulePositions);

        boolean rejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(Constants.poseEstimatorCons.shooterCamera, 
            getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.poseEstimatorCons.shooterCamera);
        setTagCount(botPose.tagCount);
        if (Math.abs(gyro.getRate()) > 720) rejectUpdate = true;
        if (botPose.tagCount == 0) rejectUpdate = true;
        if (rejectUpdate == false) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(getVisionStdDevs(), getVisionStdDevs(), 9999999));
            poseEstimator.addVisionMeasurement(botPose.pose, botPose.timestampSeconds);
        }
        
        field.setRobotPose(getPose());
    }

    @Override 
    public void periodic() {

        update();
        
        SmartDashboard.putData("Field", field);

        SmartDashboard.putNumber("Speaker Distance", getSpeakerDistance());
    }

    @Override 
    public void simulationPeriodic() {}
}
