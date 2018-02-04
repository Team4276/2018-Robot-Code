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

	TalonSRX pivotMotor;

	double armPosition = -90;
	double armSetpoint = 90;
	double armPositionError = 0;
	double armPositionErrorLast = 0;
	private double accumulatedError = 0;
	private double rateError = 0;

	private boolean manualOveride = false;
	private boolean initializePID = true;
	private double timeNow;
	private double timePrevious;
	private double timeStep;

	final double PROPORTIONAL_GAIN = 0;
	final double INTEGRAL_GAIN = 0;
	final double DERIVATIVE_GAIN = 0;
	final double TORQUE_GAIN = 0;
	final double UPPER_LIMIT = 90;
	final double LOWER_LIMIT = -90;
	final double DEGREES_PER_PULSE = (360 / 1024) / 2; // needs testing: 360
														// deg/rev | 1024
														// pulse/rev | 2:1
														// reduction

	public ArmPivoter(int pivoterCANPort) {

		pivotMotor = new TalonSRX(pivoterCANPort);

	}

	private double computeMovementPower() {
		double assignedPower;
		if (initializePID == true) {
			timeNow = Robot.systemTimer.get();
			armPosition = pivotMotor.getSensorCollection().getQuadraturePosition() * DEGREES_PER_PULSE;
			armPositionError = armSetpoint - armPosition;
			accumulatedError = 0.0;
			assignedPower = 0.0;
			initializePID = false;
		} else {
			armPositionErrorLast = armPositionError;
			timePrevious = timeNow;
			timeNow = Robot.systemTimer.get();
			armPosition = pivotMotor.getSensorCollection().getQuadraturePosition() * DEGREES_PER_PULSE;
			timeStep = timeNow - timePrevious;

			armPositionError = armSetpoint - armPosition; // proportional
			accumulatedError = accumulatedError + (armPositionErrorLast + armPositionError) / 2 * timeStep; // integral
																											// using
																											// trapezoidal
																											// approximation
			rateError = -pivotMotor.getSensorCollection().getQuadratureVelocity() * DEGREES_PER_PULSE * 10; // derivative
																											// (*10
																											// because
																											// returns
																											// counts/100ms)

			assignedPower = PROPORTIONAL_GAIN * armPositionError + INTEGRAL_GAIN * accumulatedError
					+ DERIVATIVE_GAIN * rateError;
		}
		return assignedPower;
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

	public void findSetpoint() {

		if (Robot.xboxController.getRawButton(Xbox.Start)
				&& (Math.abs(Robot.xboxController.getRawAxis(Xbox.LAxisY)) > .075)) {
			manualOveride = true;
		} else if (Robot.xboxController.getRawButton(Xbox.X)) {
			manualOveride = false;
			armSetpoint = 0;
		} else {
			manualOveride = false;
			if (Robot.xboxController.getRawAxis(Xbox.LAxisY) > .5) {
				armSetpoint++;
			} else if (Robot.xboxController.getRawAxis(Xbox.LAxisY) < -.5) {
				armSetpoint--;
			}
		}
		if (armSetpoint > UPPER_LIMIT) {
			armSetpoint = UPPER_LIMIT;
		} else if (armSetpoint < LOWER_LIMIT) {
			armSetpoint = LOWER_LIMIT;
		}

	}

	public void assignMotorPower(double staticPower, double activePower) {
		if (manualOveride) {

		} else {
			pivotMotor.set(ControlMode.PercentOutput, Robot.xboxController.getRawAxis(Xbox.LAxisY));
		}
		pivotMotor.set(ControlMode.PercentOutput, (staticPower + activePower));
	}

	public void giveReadouts() {
		SmartDashboard.putNumber("Arm Angle Commanded", armSetpoint);
		SmartDashboard.putNumber("Arm Angle Estimated", armPosition);
	}

	public void performMainProcessing() {
		findSetpoint();
		assignMotorPower(findHoldingPower(), computeMovementPower());
		giveReadouts();
	}

	public void performMainProcessing(double setPoint) {
		armSetpoint = setPoint;
		assignMotorPower(findHoldingPower(), computeMovementPower());
		giveReadouts();
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Arm Angle:", armPosition);
	}

	public void run() {
		while (true) {
			performMainProcessing();
			updateTelemetry();
			Timer.delay(0.05);
		}
	}

}
