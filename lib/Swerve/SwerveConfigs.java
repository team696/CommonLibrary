package team696.frc.lib.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class SwerveConfigs {
        public final static TalonFXConfiguration angle;
        public final static TalonFXConfiguration drive;
        public final static CANcoderConfiguration canCoder;
        public final static Pigeon2Configuration pigeon;
        public final static SwerveModuleConstants Mod0;
        public final static SwerveModuleConstants Mod1;
        public final static SwerveModuleConstants Mod2;
        public final static SwerveModuleConstants Mod3;
        static {
                angle = new TalonFXConfiguration();
                drive = new TalonFXConfiguration();
                canCoder = new CANcoderConfiguration();  
                pigeon = new Pigeon2Configuration();

                Mod0 = new SwerveModuleConstants();
                Mod1 = new SwerveModuleConstants();
                Mod2 = new SwerveModuleConstants();
                Mod3 = new SwerveModuleConstants();
                
                /** Swerve CANCoder Configuration */
                canCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                canCoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

                /** Swerve Angle Motor Configuration */
                angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                angle.Feedback.SensorToMechanismRatio = SwerveConstants.angleGearRatio;
                angle.ClosedLoopGeneral.ContinuousWrap = true;
                angle.CurrentLimits.SupplyCurrentLimitEnable = true;
                angle.CurrentLimits.SupplyCurrentLimit = 25;
                angle.CurrentLimits.SupplyCurrentThreshold = 40;
                angle.CurrentLimits.SupplyTimeThreshold = 0.1;
                angle.CurrentLimits.StatorCurrentLimitEnable = true;
                angle.CurrentLimits.StatorCurrentLimit = 40;
                angle.Slot0.kP = 175.0;
                angle.Slot0.kI = 0.0;
                angle.Slot0.kD = 0.0;

                angle.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
                angle.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

                angle.Voltage.PeakForwardVoltage = 12.;
                angle.Voltage.PeakReverseVoltage = -12.;

                /** Swerve Drive Motor Configuration */
                drive.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                drive.Feedback.SensorToMechanismRatio = SwerveConstants.driveGearRatio;
                drive.CurrentLimits.SupplyCurrentLimitEnable = true;
                drive.CurrentLimits.SupplyCurrentLimit = 25;
                drive.CurrentLimits.SupplyCurrentThreshold = 60;
                drive.CurrentLimits.SupplyTimeThreshold = 0.2;
                drive.CurrentLimits.StatorCurrentLimitEnable = true;
                drive.CurrentLimits.StatorCurrentLimit = 60;

                drive.Voltage.PeakForwardVoltage = 12.;
                drive.Voltage.PeakReverseVoltage = -12.;

                drive.Slot0.kP = 2.;
                drive.Slot0.kI = 0.0;
                drive.Slot0.kD = 0.0;
                drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
                drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
                drive.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;
                drive.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

                /** Individual Swerve Module Configurations -> frontLeft, frontRight, backLeft, backRight */ 
                Mod0.CANcoderId = 0;  
                Mod0.DriveMotorId = 3; 
                Mod0.SteerMotorId = 6;
                Mod0.CANcoderOffset = -0.313;
        
                Mod1.CANcoderId = 3; 
                Mod1.DriveMotorId = 4;
                Mod1.SteerMotorId = 7;
                Mod1.CANcoderOffset = 0.272;

                Mod2.CANcoderId = 2; 
                Mod2.DriveMotorId = 2;
                Mod2.SteerMotorId = 1;
                Mod2.CANcoderOffset = 0.372;

                Mod3.CANcoderId = 1; 
                Mod3.DriveMotorId = 5;
                Mod3.SteerMotorId = 0;
                Mod3.CANcoderOffset = 0.359;

                /** Pigeon Configuration */ 
                pigeon.MountPose.MountPoseYaw = 0;
        }
}
