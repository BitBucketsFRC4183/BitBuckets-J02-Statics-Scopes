package org.bitbuckets;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class OdometrySubsystem extends Robot {
    double trackWidthMeters = 10;
    double leftDistanceMeters = 2;
    double rightDistanceMeters = 4;
    Pose2d initialPoseMeters = new Pose2d();



    Rotation2d gyroAngle = new Rotation2d();
    DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(trackWidthMeters);
    DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(differentialDriveKinematics, gyroAngle, leftDistanceMeters, rightDistanceMeters, initialPoseMeters);


    void resetPose() {
         differentialDrivePoseEstimator.resetPosition(gyroAngle, leftDistanceMeters, rightDistanceMeters, initialPoseMeters);
    }

    Pose2d estimatedPose() {
       return differentialDrivePoseEstimator.getEstimatedPosition();
    }

    Rotation2d estimatedRotation() {
        return differentialDrivePoseEstimator.getEstimatedPosition().getRotation();
    }



}
