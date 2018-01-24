package org.usfirst.frc.team4276.mechanisms;

import org.usfirst.frc.team4276.autonomous.PositionFinder;
import org.usfirst.frc.team4276.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

public class DriveTrain {

	Joystick leftJoystick, rightJoystick;
	DoubleSolenoid gearShifter;

	VictorSPX leftMotor;
	VictorSPX rightMotor;
	VictorSPX leftMotorFollower1;
	VictorSPX rightMotorFollower1;
	VictorSPX leftMotorFollower2;
	VictorSPX rightMotorFollower2;
	PositionFinder robotLocator;

	public double leftDrivePower, rightDrivePower;
	public int highShifter = 4;
	public int lowShifter = 3;

	public boolean driveInit;

	private double accumulatedError;
	private double headingErrorLast = 0;
	
	private double timeNow;
	private double timePrevious;
	private double timeStep;

	public DriveTrain(PositionFinder robotPF, int shifterAPort, int shifterBPort, int leftCANPort, int rightCANPort,
			int leftCANPort1, int rightCANPort1, int leftCANPort2, int rightCANPort2) {

		robotLocator = robotPF;

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

		leftDrivePower = leftJoystick.getY();
		rightDrivePower = rightJoystick.getY();

	}

	public void checkForGearShift() {
		boolean shiftHi = rightJoystick.getRawButton(highShifter);
		boolean shiftLo = rightJoystick.getRawButton(lowShifter);
		boolean isHi = (gearShifter.get() == DoubleSolenoid.Value.kForward);
		boolean isLo = (gearShifter.get() == DoubleSolenoid.Value.kReverse);

		if (shiftHi) {
			gearShifter.set(DoubleSolenoid.Value.kForward);
		} else if (shiftLo) {
			gearShifter.set(DoubleSolenoid.Value.kReverse);
		}

	}

	public void setMotorSpeeds() {
		leftMotor.set(ControlMode.PercentOutput, leftDrivePower);
		rightMotor.set(ControlMode.PercentOutput, rightDrivePower);
	}

	public boolean rotateToHeading(double desiredHeading) {
		if (driveInit == true) {
			accumulatedError = 0;
			headingErrorLast = 0;
		}
		boolean status = false;
		double timeStep = Robot.systemTimer.get();
		double currentHeading = robotLocator.getHeading();
		double headingErrorCurrent = desiredHeading - currentHeading;
		accumulatedError = accumulatedError + (headingErrorCurrent + headingErrorLast) * timeStep; // calculates
																									// integral
																									// of
																									// heading
		double errorRate = (headingErrorCurrent - headingErrorLast) / timeStep; // integral

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
		return status;
	}

	public boolean rotateToCoordinate(double[] desiredCoordinateFacing) {
		if (driveInit == true) {
			accumulatedError = 0;
			headingErrorLast = 0;
		}
		double desiredHeading = Math.atan2(desiredCoordinateFacing[0], desiredCoordinateFacing[1]);
		// calculates heading needed to face coordinates based on inputed array
		
		boolean status = false;
		//return status of method (true when has reached target)
		
		double timeStep = Robot.systemTimer.get();
		double currentHeading = robotLocator.getHeading();
		double headingErrorCurrent = desiredHeading - currentHeading;
		accumulatedError = accumulatedError + (headingErrorCurrent + headingErrorLast) * timeStep;
		// calculates integral of heading
		
		double errorRate = (headingErrorCurrent - headingErrorLast) / timeStep; // integral

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
		return status;
	}

	public void performMainProcessing() {
		getJoystickValues();
		checkForGearShift();
		setMotorSpeeds();
	}
}
