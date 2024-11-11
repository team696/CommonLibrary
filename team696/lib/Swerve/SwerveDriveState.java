// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team696.lib.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class SwerveDriveState {
    public Pose2d pose;
    public ChassisSpeeds robotRelativeSpeeds;
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

    public double velocity() {
        return Math.sqrt(robotRelativeSpeeds.vxMetersPerSecond * robotRelativeSpeeds.vxMetersPerSecond + robotRelativeSpeeds.vyMetersPerSecond * robotRelativeSpeeds.vyMetersPerSecond);
    }

    public double angularVelocity() {
        return Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    public SwerveDriveState update(Pose2d pose, ChassisSpeeds speeds, double time) {
        this.timeSinceLastUpdate = time - this.timeStamp;

        this.pose = pose;
        this.robotRelativeSpeeds = speeds;
        this.timeStamp = time;

        return this;
    }

    public void publish() {
        Logger.recordOutput("Pose", this.pose);
        Logger.recordOutput("Speeds", this.robotRelativeSpeeds);
        updatePublisher.set(this.timeSinceLastUpdate);
    }
}
