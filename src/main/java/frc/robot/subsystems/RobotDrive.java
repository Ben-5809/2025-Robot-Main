package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class testing {

    public static ChassisSpeeds calculateChassisSpeeds(Pose2d robotPose, Pose2d targetPose) {
        Translation2d relativePosition = targetPose.getTranslation().minus(robotPose.getTranslation());      

        double targetAngle = targetPose.getRotation().getRadians();
        double currentAngle = robotPose.getRotation().getRadians();
        
        double velocityX = relativePosition.getX(); 
        double velocityY = relativePosition.getY();
        
        double velocityRotation = (targetAngle - currentAngle); 
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velocityX, velocityY, velocityRotation);

        return chassisSpeeds;
    }

    import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotDrive {

    private Pose2d currentPose; // Current robot pose
    private Pose2d targetPose;   // Target pose

    public RobotDrive(Pose2d currentPose, Pose2d targetPose) {
        this.currentPose = currentPose;
        this.targetPose = targetPose;
    }

    public ChassisSpeeds calculateChassisSpeeds() {
        // Calculate the difference in position
        Translation2d deltaPos = targetPose.getTranslation().minus(currentPose.getTranslation());

        // Calculate target angle to face the target
        double targetAngle = Math.atan2(deltaPos.getY(), deltaPos.getX());

        // Calculate distance to target
        double distance = deltaPos.getNorm();

        // Calculate desired speed in the forward direction (x) and rotation (theta)
        // Here we'll keep it simple with a proportional controller for speed
        double robotSpeed = Math.min(distance, 1.0); // Clamp speed to a max of 1.0
        double rotationSpeed = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

        // Create and return the ChassisSpeeds object
        return new ChassisSpeeds(robotSpeed, 0, rotationSpeed);
    }

    public static void main(String[] args) {
        // Example usage
        Pose2d currentPose = new Pose2d(1.0, 1.0, new Rotation2d(0.0)); // Current pose (x, y, heading)
        Pose2d targetPose = new Pose2d(3.0, 2.0, new Rotation2d(Math.PI / 4)); // Target pose

        RobotDrive robotDrive = new RobotDrive(currentPose, targetPose);
        ChassisSpeeds speeds = robotDrive.calculateChassisSpeeds();

        // Output the calculated speeds
        System.out.println("Chassis Speeds:");
        System.out.println("vx: " + speeds.vxMetersPerSecond); // Forward speed
        System.out.println("vy: " + speeds.vyMetersPerSecond); // Strafe speed (not used here)
        System.out.println("omega: " + speeds.omegaRadiansPerSecond); // Rotation speed
    }
}
}