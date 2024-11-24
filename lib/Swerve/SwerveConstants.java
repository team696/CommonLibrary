package frc.team696.lib.Swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Torque;
import frc.team696.lib.Logging.PLog;

/**
 * Constants of the Swerve Drive
 * 
 * <p> Should Be Updated For Each Robot Accordingly
 */
public class SwerveConstants {
		public static final String canBus = "rio";

		public static final int expectedModuleCount = 4;

		public static final double drivekS = (0.667 / 12); 
		public static final double drivekV = (2.44 / 12);
		public static final double drivekA = (0.27 / 12);

		public static final double DRIVE_GEAR_RATIO = (5.36 / 1.0);
		public static final double ANGLE_GEAR_RATIO = (150.0/7.0 / 1.0); 
	
		public static final Mass MASS = Pounds.of(150);

		// Distance From The Center To The Wheel, From The Distance Between Two Wheels divided By 2
		public static final Distance WHEELBASE_X = Inches.of(13.0).div(2);
		public static final Distance WHEELBASE_Y = Inches.of(13.0).div(2);
		public static final double DRIVEBASE_RADIUS_M = Math.sqrt(Math.pow(WHEELBASE_X.magnitude(), 2) + Math.pow(WHEELBASE_Y.magnitude(), 2));
		public static final Distance WHEEL_DIAMETER = Inches.of(3.94);
		public static final Distance WHEEL_RADIUS = WHEEL_DIAMETER.div(2);
		public static final Distance WHEEL_CIRCUM = WHEEL_DIAMETER.times(Math.PI);

		public static final Per<DistanceUnit, AngleUnit> DISTANCE_PER_ROTATION = WHEEL_CIRCUM.div(DRIVE_GEAR_RATIO).div(Rotations.of(1));

		// Use Reca.lc to find Stall Torque At Motor Current Limit
		// Values For Kraken X60
		public static final AngularVelocity MAX_FREESPIN_VELOCITY = Rotations.of(6000).per(Minute);
		public static final Torque MAX_MOTOR_STALL_TORQUE = NewtonMeters.of(1.133); 
		public static final Force MAX_WHEEL_FORCE = MAX_MOTOR_STALL_TORQUE.times(DRIVE_GEAR_RATIO).div(WHEEL_RADIUS).times(expectedModuleCount);

		public static final LinearVelocity THEORETICAL_MAX_SPEED = MetersPerSecond.of(MAX_FREESPIN_VELOCITY.in(RotationsPerSecond) * DISTANCE_PER_ROTATION.in(Meters.per(Rotation))); 
		public static final LinearAcceleration THEORETICAL_MAX_ACCELERATION = (MAX_WHEEL_FORCE.div(MASS));  
		public static final LinearVelocity MAX_VELOCITY = THEORETICAL_MAX_SPEED.times(0.85); 
		public static final LinearAcceleration MAX_ACCELERATION = THEORETICAL_MAX_ACCELERATION.times(0.7); 
		public static final AngularVelocity THEORETICAL_MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(THEORETICAL_MAX_SPEED.in(MetersPerSecond) / (DRIVEBASE_RADIUS_M * 2 * Math.PI));
		public static final AngularVelocity MAX_ANGULAR_VELOCITY = THEORETICAL_MAX_ANGULAR_VELOCITY.times(0.85); 

		public static final Translation2d[] modPositions = {
			new Translation2d(WHEELBASE_X, WHEELBASE_Y), // FL
			new Translation2d(WHEELBASE_X, WHEELBASE_Y.times(-1)), // FR
			new Translation2d(WHEELBASE_X.times(-1), WHEELBASE_Y), // BL
			new Translation2d(WHEELBASE_X.times(-1), WHEELBASE_Y.times(-1)) // BR  
		};

}
