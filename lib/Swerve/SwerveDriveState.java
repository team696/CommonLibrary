// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team696.lib.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

/** 
 * Holds the current State of the robot and last update.
 * 
 * <p> Should Not be manually updated
 */
public class SwerveDriveState {
    public Pose2d pose;
    public ChassisSpeeds robotRelativeSpeeds;
    public ChassisSpeeds robotAcceleration = new ChassisSpeeds();
    public double timeStamp;
    public double timeSinceLastUpdate;

    private final static DoublePublisher updatePublisher = NetworkTableInstance.getDefault().getDoubleTopic("696/RobotState/loopTime").publish();

    public SwerveDriveState(Pose2d pose, ChassisSpeeds speeds, double time) {
        update(pose, speeds, time);
    }

    public SwerveDriveState(SwerveDriveState other) {
        this(other.pose, other.robotRelativeSpeeds, other.timeStamp);
        this.timeSinceLastUpdate = other.timeSinceLastUpdate;
    }

    public SwerveDriveState() {
        this(new Pose2d(), new ChassisSpeeds(), 0);
    }

    /**
     * 
     * @return Current X,Y velocity of the robot in M/S
     */
    public double speed() {
        return Math.sqrt(robotRelativeSpeeds.vxMetersPerSecond * robotRelativeSpeeds.vxMetersPerSecond + robotRelativeSpeeds.vyMetersPerSecond * robotRelativeSpeeds.vyMetersPerSecond);
    }

    /**
     * 
     * @return Current X,Y velocity of robot in M/S as a Vector
     */
    public Translation2d velocityXY() {
        return new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
    }

    public Translation2d accelerationXY() {
        return new Translation2d(robotAcceleration.vxMetersPerSecond, robotAcceleration.vyMetersPerSecond);
    }

    /**
     * 
     * @return Current Rotational Velocity of the robot in Rad/s
     */
    public double angularVelocity() {
        return Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Used interally for updated estimation
     * 
     * @param pose
     * @param speeds
     * @param time
     * @return
     */
    public SwerveDriveState update(Pose2d pose, ChassisSpeeds speeds, double time) {
        this.timeSinceLastUpdate = time - this.timeStamp;
        if (timeSinceLastUpdate == 0) timeSinceLastUpdate = 1e-6;
        this.robotAcceleration.vxMetersPerSecond = (speeds.vxMetersPerSecond - this.robotRelativeSpeeds.vxMetersPerSecond) / this.timeSinceLastUpdate;
        this.robotAcceleration.vyMetersPerSecond = (speeds.vyMetersPerSecond - this.robotRelativeSpeeds.vyMetersPerSecond) / this.timeSinceLastUpdate;
        this.robotAcceleration.omegaRadiansPerSecond = (speeds.omegaRadiansPerSecond - this.robotRelativeSpeeds.omegaRadiansPerSecond) / this.timeSinceLastUpdate;

        this.pose = pose;
        this.robotRelativeSpeeds = speeds;
        this.timeStamp = time;

        return this;
    }

    /**
     * Logs the state of the robot 
     */
    public void publish() {
        Logger.recordOutput("Pose", this.pose);
        Logger.recordOutput("Speeds", this.robotRelativeSpeeds);
        updatePublisher.set(this.timeSinceLastUpdate);
    }
}
 

/**
 *  Potential Future work -> Incorporate Gyro Readings into Current Acceleration and Store Jerk, Might be cool
 *
 *  Add system for keeping track of confidence of current pose (1690). Overtime accumulate error from odometry -> reset down when getting vision readings ect.
 * 
 * Will help update vision faster and more reliably, don't need to trust shit vision poses when we have gotten a good one and barely moved! 
 */