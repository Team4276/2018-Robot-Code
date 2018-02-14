package org.usfirst.frc.team4276.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.Xbox;

/**
 * 
 * Placeholding values at lines: 43,88,90,91
 * 
 * Experimental values at lines: 39,40,41,42
 * 
 * @author Avery
 *
 */

public class ArmPivoter extends Thread implements Runnable {

	TalonSRX pivoter;

	double armPosition = -90;
	double armSetpoint = 90;
	double positionError = 0;
	double armPositionErrorLast = 0;
	private double accumulatedError = 0;
	private double rateError = 0;

	private boolean manualOveride = false;
	private boolean initializePID = true;
	private double timeNow;
	private double timePrevious;
	private double timeStep;

	double PROPORTIONAL_GAIN = 0;
	double INTEGRAL_GAIN = 0;
	double DERIVATIVE_GAIN = 0;
	final double TORQUE_GAIN = 0;
	final double UPPER_LIMIT = 90;
	final double LOWER_LIMIT = -90;
	final double DEGREES_PER_PULSE = (360 / 1024) / 2; // needs testing: 360
														// deg/rev | 1024
														// pulse/rev | 2:1
														// reduction

	public ArmPivoter(int pivoterCANPort) {
		pivoter = new TalonSRX(pivoterCANPort);
	}

	private double computeMovementPower() {
		double assignedPower;
		if (initializePID == true) {
			timeNow = Robot.systemTimer.get();
			armPosition = pivoter.getSensorCollection().getQuadraturePosition() * DEGREES_PER_PULSE;
			positionError = armSetpoint - armPosition;
			accumulatedError = 0.0;
			assignedPower = 0.0;
			initializePID = false;
		} else {
			armPositionErrorLast = positionError;
			timePrevious = timeNow;
			timeNow = Robot.systemTimer.get();
			armPosition = pivoter.getSensorCollection().getQuadraturePosition() * DEGREES_PER_PULSE;
			timeStep = timeNow - timePrevious;

			positionError = armSetpoint - armPosition; // deg
			accumulatedError = accumulatedError + (armPositionErrorLast + positionError) / 2 * timeStep; // deg*s
			rateError = -pivoter.getSensorCollection().getQuadratureVelocity() * DEGREES_PER_PULSE * 10; // deg/s

			assignedPower = PROPORTIONAL_GAIN * positionError + INTEGRAL_GAIN * accumulatedError
					+ DERIVATIVE_GAIN * rateError;
		}
		return assignedPower;
	}

	public void tuneControlGains() {
		if (Robot.logitechJoystickL.getRawButton(7) == true) {
			PROPORTIONAL_GAIN = PROPORTIONAL_GAIN + 1;
		}
		if (Robot.logitechJoystickL.getRawButton(8) == true) {
			PROPORTIONAL_GAIN = PROPORTIONAL_GAIN - 1;
		}
		if (Robot.logitechJoystickL.getRawButton(9) == true) {
			INTEGRAL_GAIN = INTEGRAL_GAIN + 1;
		}
		if (Robot.logitechJoystickL.getRawButton(10) == true) {
			INTEGRAL_GAIN = INTEGRAL_GAIN - 1;
		}
		if (Robot.logitechJoystickL.getRawButton(11) == true) {
			DERIVATIVE_GAIN = DERIVATIVE_GAIN + 1;
		}
		if (Robot.logitechJoystickL.getRawButton(12) == true) {
			DERIVATIVE_GAIN = DERIVATIVE_GAIN - 1;
		}
		SmartDashboard.putNumber("Pivoter Kp", PROPORTIONAL_GAIN);
		SmartDashboard.putNumber("Pivoter Ki", INTEGRAL_GAIN);
		SmartDashboard.putNumber("Pivoter Kd", DERIVATIVE_GAIN);
	}

	public double findHoldingPower() {
		double assignedPower = 0;

		/**
		 * torque = m*(g+a)*Xcom*cos(theta)
		 * 
		 * assuming a = 0 for now
		 * 
		 */

		double mass = 6.8; // placeholder
		double gravity = 9.8;
		double acceleration = 0; // placeholder
		double xCM = .203; // placeholder
		double theta = Math.PI * armPosition / 180;

		assignedPower = TORQUE_GAIN * (mass * (gravity + acceleration) * xCM * Math.cos(theta));

		return assignedPower;
	}

	public void determineSetpoint() {

		// Manual override
		if (Robot.xboxController.getRawButton(Xbox.Start)
				&& (Math.abs(Robot.xboxController.getRawAxis(Xbox.LAxisY)) > 0.075)) {
			manualOveride = true;
			return;
		}

		// Determine setpoint
		if (Robot.xboxController.getRawButton(Xbox.X)) {
			manualOveride = false;
			armSetpoint = 0;
		} else if (Robot.xboxController.getRawAxis(Xbox.LAxisY) < -0.5) {
			manualOveride = false;
			armSetpoint++;
		} else if (Robot.xboxController.getRawAxis(Xbox.LAxisY) > 0.5) {
			manualOveride = false;
			armSetpoint--;
		}

		// Limit setpoint range
		if (armSetpoint > UPPER_LIMIT) {
			armSetpoint = UPPER_LIMIT;
		} else if (armSetpoint < LOWER_LIMIT) {
			armSetpoint = LOWER_LIMIT;
		}

	}

	public void assignMotorPower(double staticPower, double activePower) {
		if (manualOveride) {
			pivoter.set(ControlMode.PercentOutput, -Robot.xboxController.getRawAxis(Xbox.LAxisY));
		} else {
			pivoter.set(ControlMode.PercentOutput, (staticPower + activePower));
		}
	}

	public void commandSetpoint(double setPoint) {
		armSetpoint = setPoint;
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Arm Angle Commanded", armSetpoint);
		SmartDashboard.putNumber("Arm Angle Estimated", armPosition);
	}

	public void run() {
		while (true) {
			tuneControlGains();
			determineSetpoint();
			assignMotorPower(findHoldingPower(), computeMovementPower());
			updateTelemetry();
			Timer.delay(0.05);
		}
	}

}
