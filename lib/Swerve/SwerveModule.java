package team696.frc.lib.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team696.frc.lib.Util;
import team696.frc.lib.HardwareDevices.CANCoderFactory;
import team696.frc.lib.HardwareDevices.TalonFactory;

public class SwerveModule implements Sendable{
    public static int s_moduleCount = 0;

    private final StatusSignal<Double> _encoderSignal;
    private final StatusSignal<Double> _anglePositionSignal;
    private final StatusSignal<Double> _angleVelocitySignal;
    private final StatusSignal<Double> _drivePositionSignal;
    private final StatusSignal<Double> _driveVelocitySignal;

    public static final int SIGNAL_COUNT = 4; 

    public int moduleNumber;
    private double _angleOffset;
    
    private TalonFactory _angleMotor;
    private TalonFactory _driveMotor;
    private CANCoderFactory _encoder;

    private double _lastAngle;

    private final SimpleMotorFeedforward _driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.drivekS, SwerveConstants.drivekV, SwerveConstants.drivekA);
    
    private final VelocityVoltage _driveVelocity = new VelocityVoltage(0);

    private final PositionVoltage _anglePosition = new PositionVoltage(0);

    public SwerveModule(SwerveModuleConstants moduleConstants){
        this.moduleNumber = s_moduleCount++;
        this._angleOffset = moduleConstants.CANcoderOffset; 
        /* Angle Encoder Config */
        _encoder = new CANCoderFactory(moduleConstants.CANcoderId, SwerveConstants.canBus, SwerveConfigs.canCoder, "Swerve Encoder " + moduleNumber);

        _encoderSignal = ( _encoder.get().getAbsolutePosition()); 

        /* Angle Motor Config */
        _angleMotor = new TalonFactory(moduleConstants.SteerMotorId, SwerveConstants.canBus, SwerveConfigs.angle, "Swerve Angle " + moduleNumber);
        resetToAbsolute();

        /* Drive Motor Config */
        _driveMotor = new TalonFactory(moduleConstants.DriveMotorId, SwerveConstants.canBus, SwerveConfigs.drive, "Swerve Drive " + moduleNumber);

        _anglePositionSignal = (_angleMotor.get().getPosition());
        _angleVelocitySignal = (_angleMotor.get().getVelocity());
        _drivePositionSignal = (_driveMotor.get().getPosition());
        _driveVelocitySignal = (_driveMotor.get().getVelocity());
  
        BaseStatusSignal.setUpdateFrequencyForAll(100, _encoderSignal, _anglePositionSignal, _angleVelocitySignal, _drivePositionSignal, _driveVelocitySignal);

        ParentDevice.optimizeBusUtilizationForAll(_encoder.get(), _angleMotor.get(), _driveMotor.get());

        _lastAngle = getState().angle.getRotations();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        double angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01)) ? _lastAngle : desiredState.angle.getRotations());
        _angleMotor.setControl(_anglePosition.withPosition(angle));
        setSpeed(desiredState, isOpenLoop);
        _lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        double ratio = Math.cos(desiredState.angle.getRadians() - getState().angle.getRadians()); 
        if(isOpenLoop){
            _driveMotor.VoltageOut(desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed * ratio);
        } else {
            _driveVelocity.Velocity = Util.MPSToRPS(desiredState.speedMetersPerSecond * ratio, SwerveConstants.wheelCircumference);
            _driveVelocity.FeedForward = _driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            _driveMotor.setControl(_driveVelocity);
        }
    }

    public Rotation2d getCANCoderAngle(boolean refresh){
        if (refresh) {
            _encoderSignal.refresh();
        }

        return Rotation2d.fromRotations(_encoderSignal.getValueAsDouble());
    }

    public Rotation2d getCANCoderAngle(){
        return getCANCoderAngle(true);
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANCoderAngle(true).getRotations() - _angleOffset;
        _angleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(boolean refresh){
        if (refresh) {
            _driveVelocitySignal.refresh();
        }
        return new SwerveModuleState(
            Util.RPSToMPS(_driveVelocitySignal.getValueAsDouble(), SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(getAngleMotorPosition(refresh))
        );
    }

    public SwerveModuleState getState(){
        return getState(false);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Util.rotationsToMeters(getDriveMotorPosition(), SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(getAngleMotorPosition())
        );
    }

    public double getDriveMotorPosition(boolean refresh) {
        if (refresh) {
            _drivePositionSignal.refresh();
            _driveVelocitySignal.refresh();
        }
        return BaseStatusSignal.getLatencyCompensatedValue(_drivePositionSignal, _driveVelocitySignal);
    }

    public double getDriveMotorPosition() {
        return getDriveMotorPosition(false);
    }

    public double getAngleMotorPosition(boolean refresh) {
        if (refresh) {
            _anglePositionSignal.refresh();
            _angleVelocitySignal.refresh();
        }
        return BaseStatusSignal.getLatencyCompensatedValue(_anglePositionSignal, _angleVelocitySignal);
    }

    public double getAngleMotorPosition() {
        return getAngleMotorPosition(false);
    }

    public BaseStatusSignal[] getSignals(){
        return new BaseStatusSignal[] {_anglePositionSignal, _angleVelocitySignal, _drivePositionSignal, _driveVelocitySignal, _encoderSignal};
    }

    public void putData() {
        SmartDashboard.putData("Swerve/Mod " + this.moduleNumber, this);
    }

    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Motor Angle " + moduleNumber, this::getAngleMotorPosition, null);
        builder.addDoubleProperty("Velocity " + moduleNumber, this::getDriveMotorPosition, null);
        builder.addDoubleProperty("Encoder Angle " + moduleNumber, ()->this.getCANCoderAngle().getRotations(), null);
    }
}