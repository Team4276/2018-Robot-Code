package mechanisms;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4276.robot.Robot;
import utilities.Xbox;

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
			accumulatedError = accumulatedError + armPositionErrorLast * timeStep; // integral
			armErrorRateOfChange = pivotMotor.getSensorCollection().getQuadratureVelocity(); // derivative

			assignedPower = proportionalGain * armPositionError + integralGain * accumulatedError
					+ derivativeGain * armErrorRateOfChange;
		}
		return assignedPower;
	}

	public void findSetPoint() {

		if (Robot.xboxController.getRawButton(Xbox.X)) {
			armSetpoint = 0;
		} else {
			if (Xbox.LAxisY > .5) {
				armSetpoint++;
			} else if (Xbox.LAxisY < -.5) {
				armSetpoint--;
			}
		}

	}
	
	public void giveReadouts(){
		SmartDashboard.putNumber("Arm Angle Commanded", armSetpoint);
		SmartDashboard.putNumber("Arm Angle Estimated", armPosition);
	}

}
