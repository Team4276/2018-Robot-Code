package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.autonomous.PositionFinder;
import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.SoftwareTimer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {

	Joystick leftJoystick, rightJoystick;
	DoubleSolenoid gearShifter;

	VictorSPX leftMotor;
	VictorSPX rightMotor;
	VictorSPX leftMotorFollower1;
	VictorSPX rightMotorFollower1;
	VictorSPX leftMotorFollower2;
	VictorSPX rightMotorFollower2;

	SoftwareTimer shiftTimer;

	private boolean shiftInit = true;

	public double leftDrivePower, rightDrivePower;
	public int highShifter = 4;
	public int lowShifter = 3;
	public final double SHIFT_TIME = .05;

	public boolean driveInit = true;

	private double accumulatedError;
	private double errorLast = 0;

	private double timeNow;
	private double timePrevious;
	private double timeStep;

	public String driveMode = "init";

	public DriveTrain(int shifterAPort, int shifterBPort, int leftCANPort, int rightCANPort, int leftCANPort1,
			int rightCANPort1, int leftCANPort2, int rightCANPort2) {

		leftMotor = new VictorSPX(leftCANPort);
		rightMotor = new VictorSPX(rightCANPort);
		leftMotorFollower1 = new VictorSPX(leftCANPort1);
		rightMotorFollower1 = new VictorSPX(rightCANPort1);
		leftMotorFollower2 = new VictorSPX(leftCANPort2);
		rightMotorFollower2 = new VictorSPX(rightCANPort2);

		leftMotorFollower1.set(ControlMode.Follower, leftCANPort);
		rightMotorFollower1.set(ControlMode.Follower, rightCANPort);
		leftMotorFollower2.set(ControlMode.Follower, leftCANPort);
		rightMotorFollower2.set(ControlMode.Follower, rightCANPort);

		gearShifter = new DoubleSolenoid(shifterAPort, shifterBPort);

	}

	public void getJoystickValues() {

		leftDrivePower = Robot.logitechJoystickL.getY();
		rightDrivePower = Robot.logitechJoystickR.getY();

	}

	public void checkForGearShift() {
		boolean shiftHi = Robot.logitechJoystickR.getRawButton(highShifter);
		boolean shiftLo = Robot.logitechJoystickR.getRawButton(lowShifter);

		if (shiftHi) {
			if (shiftInit) {
				shiftTimer.setTimer(SHIFT_TIME);
				shiftInit = false;
			}
			if (shiftTimer.isExpired()) {
				setMotorSpeeds();
				shiftInit = true;
			} else {
				setMotorSpeeds(0);
			}
			gearShifter.set(DoubleSolenoid.Value.kForward);
		} else if (shiftLo) {
			if (shiftInit) {
				shiftTimer.setTimer(SHIFT_TIME);
				shiftInit = false;
			}
			if (shiftTimer.isExpired()) {
				setMotorSpeeds();
				shiftInit = true;
			} else {
				setMotorSpeeds(0);
			}
			gearShifter.set(DoubleSolenoid.Value.kReverse);
		} else {
			setMotorSpeeds();
		}

	}

	public void setMotorSpeeds() {
		leftMotor.set(ControlMode.PercentOutput, leftDrivePower);
		rightMotor.set(ControlMode.PercentOutput, rightDrivePower);
	}

	public void setMotorSpeeds(double speed) {
		leftMotor.set(ControlMode.PercentOutput, speed);
		rightMotor.set(ControlMode.PercentOutput, speed);
	}

	public boolean rotateToHeading(double desiredHeading) {
		if (driveInit == true) {
			accumulatedError = 0;
			errorLast = 0;
			driveInit = false;
		}
		boolean status = false;
		double timeStep = Robot.systemTimer.get();
		double currentHeading = PositionFinder.getHeadingDeg();
		double headingErrorCurrent = desiredHeading - currentHeading;
		accumulatedError = accumulatedError + (headingErrorCurrent + errorLast) * timeStep; // calculates
																							// integral
																							// of
																							// heading
		double errorRate = (headingErrorCurrent - errorLast) / timeStep; // integral

		final double PROPORTIONAL_GAIN = .015;
		final double INTEGRAL_GAIN = 0;
		final double POSITION_DEADBAND = 2; // degrees
		final double RATE_DEADBAND = 10; // degrees per second

		leftDrivePower = PROPORTIONAL_GAIN * headingErrorCurrent + INTEGRAL_GAIN * accumulatedError;
		rightDrivePower = -1 * (PROPORTIONAL_GAIN * headingErrorCurrent + INTEGRAL_GAIN * accumulatedError);

		if (Math.abs(headingErrorCurrent) < POSITION_DEADBAND && Math.abs(errorRate) < RATE_DEADBAND) {
			status = true;
			leftDrivePower = 0;
			rightDrivePower = 0;
		}
		setMotorSpeeds();
		driveMode = "Rotating to: " + desiredHeading;
		return status;
	}

	public void resetDrive() {
		setMotorSpeeds(0);
		driveInit = true;
	}

	public boolean rotateToCoordinate(double[] desiredCoordinateFacing) {
		if (driveInit == true) {
			accumulatedError = 0;
			errorLast = 0;
			timeNow = Robot.systemTimer.get();
			driveInit = false;
		}
		double desiredHeading = Math.atan2(desiredCoordinateFacing[1], desiredCoordinateFacing[0]);
		// calculates heading needed to face coordinates based on inputed array

		boolean status = false;
		// return status of method (true when has reached target)

		timePrevious = timeNow;
		timeNow = Robot.systemTimer.get();
		double currentHeading = PositionFinder.getHeadingDeg();
		timeStep = timeNow - timePrevious;
		double headingErrorCurrent = desiredHeading - currentHeading;
		accumulatedError = accumulatedError + (headingErrorCurrent + errorLast) * timeStep;
		// calculates integral of heading error

		double errorRate = (headingErrorCurrent - errorLast) / timeStep; // integral

		final double PROPORTIONAL_GAIN = .015;
		final double INTEGRAL_GAIN = 0;
		final double POSITION_DEADBAND = 2; // degrees
		final double RATE_DEADBAND = 10; // degrees per second

		leftDrivePower = PROPORTIONAL_GAIN * headingErrorCurrent + INTEGRAL_GAIN * accumulatedError;
		rightDrivePower = -1 * (PROPORTIONAL_GAIN * headingErrorCurrent + INTEGRAL_GAIN * accumulatedError);

		if (Math.abs(headingErrorCurrent) < POSITION_DEADBAND && Math.abs(errorRate) < RATE_DEADBAND) {
			status = true;
			leftDrivePower = 0;
			rightDrivePower = 0;
		}
		setMotorSpeeds();
		driveMode = "Pointing to: " + desiredCoordinateFacing[0] + "," + desiredCoordinateFacing[1];
		return status;
	}

	public boolean driveToCoordinate(double[] desiredCoordinate) {
		if (driveInit == true) {
			accumulatedError = 0;
			errorLast = 0;
			timeNow = Robot.systemTimer.get();
			driveInit = false;
		}
		double desiredHeading = Math.atan2(desiredCoordinate[1] - PositionFinder.getCurrentLocation()[1],
				desiredCoordinate[0] - PositionFinder.getCurrentLocation()[0]);
		// calculates heading needed to face coordinates based on inputed array

		boolean status = false;
		// return status of method (true when has reached target)

		timePrevious = timeNow;
		timeNow = Robot.systemTimer.get();
		double errorCurrent = Math.sqrt(Math.pow(desiredCoordinate[0] - PositionFinder.currentXY[0], 2)
				+ Math.pow(desiredCoordinate[1] - PositionFinder.currentXY[1], 2));
		timeStep = timeNow - timePrevious;

		double currentHeading = PositionFinder.getHeadingDeg();
		double headingError = desiredHeading - currentHeading;
		accumulatedError = accumulatedError + (errorCurrent + errorLast) * timeStep;
		// calculates integral of heading error

		double errorRate = (errorCurrent - errorLast) / timeStep; // integral

		final double LINEAR_PROPORTIONAL_GAIN = .1;
		final double LINEAR_INTEGRAL_GAIN = 0;
		final double ANGLER_PROPORTIONAL_GAIN = .015; // degrees
		final double LINEAR_DEADBAND = 0.1; // feet
		final double LINEAR_RATE_DEADBAND = .5; // feet per second

		boolean facingTarget = (Math.abs(headingError) < 90);

		if (facingTarget) {
			leftDrivePower = LINEAR_PROPORTIONAL_GAIN * errorCurrent + LINEAR_INTEGRAL_GAIN * accumulatedError
					+ ANGLER_PROPORTIONAL_GAIN * headingError;
			rightDrivePower = LINEAR_PROPORTIONAL_GAIN * errorCurrent + LINEAR_INTEGRAL_GAIN * accumulatedError
					- ANGLER_PROPORTIONAL_GAIN * headingError;
		} else {
			leftDrivePower = -1 * LINEAR_PROPORTIONAL_GAIN * headingError + LINEAR_INTEGRAL_GAIN * accumulatedError
					+ ANGLER_PROPORTIONAL_GAIN * headingError;
			rightDrivePower = -1 * LINEAR_PROPORTIONAL_GAIN * headingError + LINEAR_INTEGRAL_GAIN * accumulatedError
					- ANGLER_PROPORTIONAL_GAIN * headingError;
		}

		if (Math.abs(errorCurrent) < LINEAR_DEADBAND && Math.abs(errorRate) < LINEAR_RATE_DEADBAND) {
			status = true;
			leftDrivePower = 0;
			rightDrivePower = 0;
		}
		setMotorSpeeds();
		driveMode = "Driving to: " + desiredCoordinate[0] + "," + desiredCoordinate[1];
		return status;
	}

	public void reverse(boolean isReversing) {

		if (isReversing) {
			leftDrivePower = -.5;
			rightDrivePower = -.5;
		} else {
			leftDrivePower = -.5;
			rightDrivePower = -.5;
		}
		setMotorSpeeds();
		driveMode = "Driving Back ";
	}

	public void performMainProcessing() {
		getJoystickValues();
		checkForGearShift();
		driveMode = "Operator Control";
	}

	public void setHiGear() {
		gearShifter.set(DoubleSolenoid.Value.kForward);
	}

	public void setLoGear() {
		gearShifter.set(DoubleSolenoid.Value.kReverse);
	}

	public void updateTelemetry() {
		SmartDashboard.putBoolean("High Gear", Robot.logitechJoystickR.getRawButton(highShifter));
		SmartDashboard.putBoolean("Low Gear", Robot.logitechJoystickR.getRawButton(lowShifter));
		SmartDashboard.putBoolean("Shifting In Progress", !shiftTimer.isExpired());
		SmartDashboard.putNumber("Left Drive Power", leftDrivePower);
		SmartDashboard.putNumber("Right Drive Power", rightDrivePower);
		SmartDashboard.putString("Drive Mode", driveMode);
	}
}
