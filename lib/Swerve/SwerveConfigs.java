package frc.team696.lib.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * 
 * Device Configs of the Swerve Drive
 * 
 * <p> Only Update Device Configs if you know what you are doing.
 */
public final class SwerveConfigs {
        public final static TalonFXConfiguration angle;
        public final static TalonFXConfiguration drive;
        public final static CANcoderConfiguration canCoder;
        public final static Pigeon2Configuration pigeon;
        public final static SwerveModuleConstants FRONT_LEFT;
        public final static SwerveModuleConstants FRONT_RIGHT;
        public final static SwerveModuleConstants BACK_LEFT;
        public final static SwerveModuleConstants BACK_RIGHT;
        static {
                angle = new TalonFXConfiguration();
                drive = new TalonFXConfiguration();
                canCoder = new CANcoderConfiguration();  
                pigeon = new Pigeon2Configuration();

                FRONT_LEFT = new SwerveModuleConstants();
                FRONT_RIGHT = new SwerveModuleConstants();
                BACK_LEFT = new SwerveModuleConstants();
                BACK_RIGHT = new SwerveModuleConstants();
                
                /** Swerve CANCoder Configuration */
                canCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                canCoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

                /** Swerve Angle Motor Configuration */
                angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                angle.Feedback.SensorToMechanismRatio = SwerveConstants.ANGLE_GEAR_RATIO;
                angle.ClosedLoopGeneral.ContinuousWrap = true;
                angle.CurrentLimits.SupplyCurrentLimitEnable = true;
                angle.CurrentLimits.SupplyCurrentLimit = 25;
                angle.CurrentLimits.SupplyCurrentLowerLimit = 60;
                angle.CurrentLimits.SupplyCurrentLowerTime = 0.1;
                angle.CurrentLimits.StatorCurrentLimitEnable = true;
                angle.CurrentLimits.StatorCurrentLimit = 80;
                angle.Slot0.kP = 150.0;
                angle.Slot0.kI = 0.0;
                angle.Slot0.kD = 0.0;

                angle.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
                angle.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
                angle.Voltage.PeakForwardVoltage = 12.;
                angle.Voltage.PeakReverseVoltage = -12.;

                /** Swerve Drive Motor Configuration */
                drive.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                drive.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO;
                drive.CurrentLimits.SupplyCurrentLimitEnable = true;
                drive.CurrentLimits.SupplyCurrentLimit = 25;
                drive.CurrentLimits.SupplyCurrentLowerLimit = 90;
                drive.CurrentLimits.SupplyCurrentLowerTime = 0.2;
                drive.CurrentLimits.StatorCurrentLimitEnable = true;
                drive.CurrentLimits.StatorCurrentLimit = 110;
                drive.Voltage.PeakForwardVoltage = 12.;
                drive.Voltage.PeakReverseVoltage = -12.;

                drive.Slot0.kP = 2.;
                drive.Slot0.kI = 0.0;
                drive.Slot0.kD = 0.0;
                drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
                drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
                drive.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;
                drive.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

                /** Individual Swerve Module Configurations */ 
                FRONT_LEFT.CANcoderId = 0;  
                FRONT_LEFT.DriveMotorId = 3; 
                FRONT_LEFT.SteerMotorId = 6;
                FRONT_LEFT.CANcoderOffset = -0.313;
        
                FRONT_RIGHT.CANcoderId = 3; 
                FRONT_RIGHT.DriveMotorId = 4;
                FRONT_RIGHT.SteerMotorId = 7;
                FRONT_RIGHT.CANcoderOffset = 0.272;

                BACK_LEFT.CANcoderId = 2; 
                BACK_LEFT.DriveMotorId = 2;
                BACK_LEFT.SteerMotorId = 1;
                BACK_LEFT.CANcoderOffset = 0.372;

                BACK_RIGHT.CANcoderId = 1; 
                BACK_RIGHT.DriveMotorId = 5;
                BACK_RIGHT.SteerMotorId = 0;
                BACK_RIGHT.CANcoderOffset = 0.359;

                /** Pigeon Configuration */ 
                pigeon.MountPose.MountPoseYaw = 0;
        }
}
