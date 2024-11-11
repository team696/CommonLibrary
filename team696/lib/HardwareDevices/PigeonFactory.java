package frc.team696.lib.HardwareDevices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.team696.lib.Logging.PLog;

public class PigeonFactory implements GyroInterface {

    private final double TIMEOUT = 0.05;
    private Pigeon2 _gyro;
    private Pigeon2Configuration _config;
    private String _name;

    private boolean _configured = false;
    private double _lastConfiguration = -100;

    public StatusSignal<Double> _yawSignal;
    public StatusSignal<Double> _yawVelocitySignal;

    public PigeonFactory(int id, String canBus, Pigeon2Configuration config, String name) {
        this._gyro = new Pigeon2(id, canBus);
        this._name = name;
        this._config = config;
        configure();
    }

    public PigeonFactory(int id, Pigeon2Configuration config, String name) {
        this(id, "rio", config, name);
    }

    private boolean configure() {
        return configure(false);
    }

        public boolean configure(boolean force) {
        if (!force && _configured) return true;
        if (!force && Timer.getFPGATimestamp() - _lastConfiguration < 3) return false;

        _lastConfiguration = Timer.getFPGATimestamp();
        StatusCode configCode = _gyro.getConfigurator().apply(this._config, TIMEOUT);

        _yawSignal = _gyro.getYaw();
        _yawVelocitySignal = _gyro.getAngularVelocityZWorld();

        if(configCode.isError() || _yawSignal.getStatus().isError() || _yawVelocitySignal.getStatus().isError()) {
            PLog.unusual(_name, "Failed to configure");
        } else {
            PLog.info(_name, "Configured");
            if (configCode.isWarning()) {
                PLog.unusual(_name, "Config Warning: " + configCode.toString());
            }

            _configured = true;
        }

        // Actually Used In Swerve
        _yawSignal.setUpdateFrequency(100);
        _yawVelocitySignal.setUpdateFrequency(100);

        // Useful Other Commands That May Be Used In The Future
        _gyro.getPitch().setUpdateFrequency(100);
        _gyro.getRoll().setUpdateFrequency(100);

        _gyro.optimizeBusUtilization();

        return _configured;
    }

    private double getRawYaw(boolean refresh) {
        if (configure()) {
            if (refresh) {
                StatusSignal<Double> code = _yawSignal.refresh();
                if(!code.getStatus().isOK()) {
                    _configured = false;
                    return 0;
                }
            }
            return MathUtil.inputModulus(_yawSignal.getValueAsDouble(),-180,180);
        } else 
            return 0;
    }

    /*
     * Angle Of the Gyro in -180, 180
     */
    @Override 
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(getRawYaw(false));
    }

    @Override 
    public void resetYaw() {
        _gyro.reset();
    }

    public double getAngularVelocity(boolean refresh) {
        if (configure()) {
            if (refresh) {
                StatusSignal<Double> code = _yawVelocitySignal.refresh();
                if(!code.getStatus().isOK()) {
                    _configured = false;
                    return 0;
                }
            }
            return _yawVelocitySignal.getValueAsDouble();
        } else 
            return 0;
    }

    @Override
    public double getAngularVelocity() {
        return getAngularVelocity(false);
    }

    public Pigeon2 get() {
        return _gyro;
    }

    public Rotation2d getLatencyAdjustedYaw() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(BaseStatusSignal.getLatencyCompensatedValue(_yawSignal, _yawVelocitySignal),-180,180));
    }
}
