package mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4276.robot.Robot;
import utilities.Xbox;

/**
 * 
 * Placeholding values at lines: 43,88,90,91
 * 
 * Experimental values at lines: 39,40,41,42
 * 
 * @author Avery
 *
 */

public class armPivoter {

	TalonSRX pivotMotor;

	double armPosition = -90;
	double armSetpoint = -90;
	double armPositionError = 0;
	double armPositionErrorLast = 0;
	double accumulatedError = 0;
	double armErrorRateOfChange = 0;

	boolean initializePID = true;
	double timeNow;
	double timePrevious;
	double timeStep;

	final double proportionalGain = 0;
	final double integralGain = 0;
	final double derivativeGain = 0;
	final double torqueGain = 0;
	final double UPPER_LIMIT = 90;
	final double LOWER_LIMIT = -90;
	final double DISTANCE_PER_PULSE = 1; // placeholder

	public armPivoter(int pivoterCANPort) {

		pivotMotor = new TalonSRX(pivoterCANPort);

	}

	private double computeMovementPower() {
		double assignedPower = 0;
		armPosition = pivotMotor.getSensorCollection().getQuadraturePosition();
		if (initializePID == true) {
			timeNow = Robot.systemTimer.get();
			armPosition = pivotMotor.getSensorCollection().getQuadraturePosition();
			armPositionError = armSetpoint - armPosition;
			accumulatedError = 0.0;
			assignedPower = 0.0;
			initializePID = false;
		} else {
			armPositionErrorLast = armPositionError;
			timePrevious = timeNow;

			timeNow = Robot.systemTimer.get();
			timeStep = timeNow - timePrevious;

			armPositionError = armSetpoint - armPosition; // proportional
			accumulatedError = accumulatedError + (armPositionErrorLast + armPositionError) / 2 * timeStep; // integral
																											// using
																											// trapezoidal
																											// approximation
			armErrorRateOfChange = pivotMotor.getSensorCollection().getQuadratureVelocity(); // derivative

			assignedPower = proportionalGain * armPositionError + integralGain * accumulatedError
					+ derivativeGain * armErrorRateOfChange;
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
		double Xcom = .203; // placeholder
		double theta = Math.PI * armPosition / 180;

		assignedPower = torqueGain * (mass * (gravity + acceleration) * Xcom * Math.cos(theta));

		return assignedPower;
	}

	public void findSetpoint() {

		if (Robot.xboxController.getRawButton(Xbox.X)) {
			armSetpoint = 0;
		} else {
			if (Xbox.LAxisY > .5) {
				armSetpoint++;
			} else if (Xbox.LAxisY < -.5) {
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

}
