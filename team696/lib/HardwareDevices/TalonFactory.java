package frc.team696.lib.HardwareDevices;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import frc.team696.lib.Logging.PLog;

public class TalonFactory { 
    private final double TIMEOUT = 0.05;
    private TalonFX _motor;
    private TalonFXConfiguration _config;
    private String _name;
    private DutyCycleOut _DutyCycleControl;
    private VoltageOut _VoltageControl;

    private int _followID = -1;
    private boolean _opposeDirection = false;

    private boolean _configured = false;
    private double _lastConfiguration = -100;

    public TalonFactory(int id, String canBus, TalonFXConfiguration config, String name) {
        this._motor = new TalonFX(id, canBus);
        this._name = name;
        this._config = config;
        _DutyCycleControl = new DutyCycleOut(0);
        _VoltageControl = new VoltageOut(0);
        configure();
    }

    public TalonFactory(int id, TalonFXConfiguration config, String name) {
        this(id, "rio", config, name);
    }

    private boolean configure() {
        return configure(false);
    }

    public boolean configure(boolean force) {
        if (!force && _configured) return true;
        if (!force && Timer.getFPGATimestamp() - _lastConfiguration < 3) return false;

        _lastConfiguration = Timer.getFPGATimestamp();
        StatusCode configCode = _motor.getConfigurator().apply(this._config, TIMEOUT);

        if(configCode.isError()) {
            PLog.unusual(_name, "Failed to configure");
        } else {
            PLog.info(_name, "Configured");
            if (configCode.isWarning()) {
                PLog.unusual(_name, "Config Warning: " + configCode.toString());
            }

            if (_followID != -1)
                _motor.setControl(new Follower(_followID, _opposeDirection));

            setPosition(0);

            _configured = true;
        }

        return _configured;
    }

    public void Follow(int id, boolean opposeDirection) {
        _followID = id;
        this._opposeDirection = opposeDirection;
        if (configure())
            _motor.setControl(new Follower(id, opposeDirection)); 
    }

    public void Follow(TalonFX master, boolean opposeDirection) {
        Follow(master.getDeviceID(), opposeDirection);
    }

    public void Follow(TalonFactory master, boolean opposeDirection) {
        Follow(master.getID(), opposeDirection);
    }

    public int getID() {
        return _motor.getDeviceID();
    }

    public void setPosition(double newPosition) {
        if (configure())
            if(!_motor.setPosition(newPosition).isOK()) {
                PLog.info(this._name, "Failed To Set Position");
                _configured = false;
            }
    }

    public void setPosition() {
        setPosition(0);
    }

    public double getPosition() {
        if (configure()) {
            StatusSignal<Double> positionCode = _motor.getPosition();
            if(!positionCode.getStatus().isOK()) {
                _configured = false;
                return 0;
            }
            return positionCode.getValueAsDouble();
        } else 
            return 0;
    }

    public double getVelocity() {
        if (configure()) {
            StatusSignal<Double> velocityCode = _motor.getVelocity();
            if(!velocityCode.getStatus().isOK()) {
                _configured = false;
                return 0;
            }
            return velocityCode.getValueAsDouble();
        } else 
            return 0;
    }

    public void PercentOutput(double percent) {
        setControl(_DutyCycleControl.withOutput(percent));
    }

    /** @param voltage 0-1 output */
    public void VoltageOut(double voltage) {
        setControl(_VoltageControl.withOutput(voltage * 12));
    }

    public void stop() {
        if (configure())
            _motor.stopMotor();
    }
    
    public void setControl(ControlRequest request) {
        if (configure())
            if (!_motor.setControl(request).isOK()) {
                _configured = false;
            }
    }

    public double getCurrent() {
        if (configure()) {
            StatusSignal<Double> currentCode = _motor.getStatorCurrent();
            if(!currentCode.getStatus().isOK()) {
                _configured = false;
                return 0;
            }
            return currentCode.getValueAsDouble();
        } else 
            return 0;
    }

    public TalonFX get() {
        return _motor;
    }
}
