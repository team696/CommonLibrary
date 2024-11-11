package frc.team696.lib.HardwareDevices;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroInterface {
    public abstract Rotation2d getYaw();

    public abstract void resetYaw();

    public abstract double getAngularVelocity();
}
