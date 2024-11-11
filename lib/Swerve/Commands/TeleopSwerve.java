package team696.frc.lib.Swerve.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team696.frc.lib.Util;
import team696.frc.lib.Swerve.SwerveConstants;
import team696.frc.lib.Swerve.SwerveDriveSubsystem;

public class TeleopSwerve extends Command {
    protected static DoubleSupplier translation = ()->0;
    protected static DoubleSupplier strafe = ()->0;
    protected static DoubleSupplier rotation = ()->0;
    protected static double deadband = 1; // deadband for controller -> defaulted to 1 so you must config swerve
    protected static double rotationDeadband = 1;
    protected static SwerveDriveSubsystem swerveSubsystem = null;
    private static PIDController pidController = new PIDController(0.0056, 0.00, 0); 
    static {
        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(1);
    }
    private static BooleanSupplier lockRotation; // should lock rotation -> usually a button

    private boolean fieldRelative; // should do fieldRelative controol
    private boolean openLoop; // should do openLoop control
    private Supplier<Rotation2d> rotationGoal; // Rotation to lock to once lockRotation has been activated
    private DoubleSupplier multiplier = ()->1; // Multiplier to outputted Speed

    public static void config(SwerveDriveSubsystem s, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, BooleanSupplier rotationLock, double deadBand) {
        strafe = x;
        translation = y;
        rotation = r;

        lockRotation = rotationLock;

        deadband = deadBand;
        rotationDeadband = Math.sqrt(2 * Math.pow(deadband, 2));
        swerveSubsystem = s;
    }

    public static TeleopSwerve New(){
        return new TeleopSwerve();
    }
    
    private TeleopSwerve() {
        this.fieldRelative = true;
        this.openLoop = true;

        this.rotationGoal = ()->null;

        this.multiplier = ()->1;

        addRequirements(swerveSubsystem);
    }

    public TeleopSwerve withMultiplier(DoubleSupplier multiplier) {
        this.multiplier = multiplier;
        return this;
    }

    public TeleopSwerve withMultiplier(double multiplier) {
        this.multiplier = ()->multiplier;
        return this;
    }

    public TeleopSwerve withRotationGoal(Supplier<Rotation2d> goal) {
        this.rotationGoal = goal;
        return this;
    }

    public TeleopSwerve withfieldRelative(boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
        return this;
    }

    public TeleopSwerve withOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
        return this;
    }

    @Override
    public void execute() {
        double yAxis = translation.getAsDouble();
        double xAxis = strafe.getAsDouble();
        double rAxis = rotation.getAsDouble();

        Rotation2d theta = new Rotation2d(yAxis, xAxis);
        double magnitude = Math.min(Math.sqrt((xAxis * xAxis) + (yAxis * yAxis)), 1);
        if (magnitude < deadband) magnitude = 0;
        Rotation2d goalRotation = rotationGoal.get();
        if (lockRotation != null && lockRotation.getAsBoolean() && goalRotation != null) { // Rotation Lock To Angle TODO: REWORK THIS PID
            double pid = pidController.calculate(swerveSubsystem.getPose().getRotation().getDegrees(), goalRotation.getDegrees());
            rAxis = Math.abs(pidController.getPositionError()) > 1 ? Math.abs(Math.pow(pid, 2)) * 1.1 * Math.signum(pid) + pid * 2.2 : 0;
        } else {
            rAxis = (Math.abs(rAxis) > rotationDeadband) ? Util.map(rAxis * rAxis, rotationDeadband, 1, 0, 1) * Math.signum(rAxis) : 0;
        }

        double outputPercent = Math.min(multiplier.getAsDouble(),1);

        double rotation = rAxis * SwerveConstants.maxAngularVelocity;
        Translation2d translation = new Translation2d(Math.pow(magnitude, 2), theta).times(SwerveConstants.maxSpeed).times(outputPercent);

        swerveSubsystem.Drive(translation, rotation, fieldRelative, openLoop);
    }
}