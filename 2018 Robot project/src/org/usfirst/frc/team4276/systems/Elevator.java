package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.Xbox;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Thread implements Runnable {

	TalonSRX elevatorDriverMain;
	VictorSPX elevatorDriverFollow;

	final double POWER_OFFSET_MANIPULATOR = 4;
	final double POWER_OFFSET_RAIL = 7;
	final double KP_LOWER = 3;
	final double KI_LOWER = 2;
	final double KD_LOWER = 1;
	final double KP_UPPER = 5;
	final double KI_UPPER = 5;
	final double KD_UPPER = 5;
	final double LOWER_HEIGHT = 10;
	final double RAIL_HEIGHT = 5;
	public final double SETPOINT_SCALE = 7;
	public final double SETPOINT_SWITCH = 3;
	public final double SETPOINT_BOTTOM = 1;
	final double SETPOINT_INCREMENT = .05; // units height
	final double OVERRIDE_INCREMENT = 0.05; // 5%

	private boolean manualOverride;
	private boolean initializePID = true;
	private double timeNow;
	private double timePrevious;
	private double timeStep;
	private double accumulatedError = 0;
	private double rateError = 0;

	double estimatedHeight = 0;
	public double commandedHeight = 0;
	double heightError = 0;
	double heightErrorLast = 0;
	double motorPower = 0;
	double assignedPower;
	double HEIGHT_PER_PULSE = 1; //

	final double initiliazedPoint = 0;
	final double derivativePoint = 0;

	public Elevator(int elevator1CANPort, int elevator2CANPort) {

		elevatorDriverMain = new TalonSRX(elevator1CANPort);
		elevatorDriverFollow = new VictorSPX(elevator2CANPort);

		elevatorDriverFollow.set(ControlMode.Follower, elevator1CANPort);

	}

	public void performMainProcessing() {

		// Start + Right Axis Y (Manually control Height by Power)
		if (Robot.xboxController.getRawButton(Xbox.Start)
				&& (Math.abs(Robot.xboxController.getRawAxis(Xbox.RAxisY)) > 0.5)) {
			manualOverride = true;
		} else {
			manualOverride = false;
		}
		
		if (manualOverride == true) {
			if (Robot.xboxController.getRawAxis(Xbox.RAxisY) < 0) {
				motorPower = motorPower + OVERRIDE_INCREMENT;
			} else {
				motorPower = motorPower - OVERRIDE_INCREMENT;
			} 
			elevatorDriverMain.set(ControlMode.PercentOutput, motorPower);
			return;
		} 

		// button Y(Deposit Cube in Scale)
		if (Robot.xboxController.getRawButton(Xbox.Y)) {
			commandedHeight = SETPOINT_SCALE;
		}

		// button A(Deposit Cube in Switch)
		if (Robot.xboxController.getRawButton(Xbox.A)) {
			commandedHeight = SETPOINT_SWITCH;
		}

		// button B(Manipulator to Bottom)
		if (Robot.xboxController.getRawButton(Xbox.B)) {
			commandedHeight = SETPOINT_BOTTOM;
		}


		// Right Axis Y (Manually Change Setpoint)
		if (Robot.xboxController.getRawAxis(Xbox.RAxisY) > 0.5) {
			commandedHeight = commandedHeight + SETPOINT_INCREMENT;
		} else if (Robot.xboxController.getRawAxis(Xbox.RAxisY) < -0.5) {
			commandedHeight = commandedHeight - SETPOINT_INCREMENT;
		}

		// Limit commanded height range
		if (commandedHeight > LOWER_HEIGHT) {
			commandedHeight = LOWER_HEIGHT;
		} else if (commandedHeight > SETPOINT_BOTTOM) {
			commandedHeight = SETPOINT_BOTTOM;
		}

		
			if (initializePID == true) {
				timeNow = Robot.systemTimer.get();
				estimatedHeight = elevatorDriverMain.getSensorCollection().getQuadraturePosition() * HEIGHT_PER_PULSE;
				heightError = commandedHeight - estimatedHeight;
				accumulatedError = 0.0;
				initializePID = false;
			} else {
				heightErrorLast = heightError;
				timePrevious = timeNow;
				timeNow = Robot.systemTimer.get();
				estimatedHeight = elevatorDriverMain.getSensorCollection().getQuadraturePosition() * HEIGHT_PER_PULSE;
				timeStep = timeNow - timePrevious;

				// Compute control errors
				heightError = commandedHeight - estimatedHeight; // height
				accumulatedError = accumulatedError + (heightErrorLast + heightError) / 2 * timeStep; // height*sec
				rateError = -elevatorDriverMain.getSensorCollection().getQuadratureVelocity() * 10 * HEIGHT_PER_PULSE; // height/sec

				// Compute PID e power
				if (commandedHeight < LOWER_HEIGHT) {
					// PID for lower rail
					motorPower = POWER_OFFSET_MANIPULATOR + (KP_LOWER * heightError) + (KI_LOWER * accumulatedError)
							+ (KD_LOWER * rateError);
				} else {
					// PID for upper rail
					motorPower = POWER_OFFSET_RAIL + (KP_UPPER * heightError) + (KI_UPPER * accumulatedError)
							+ (KD_UPPER * rateError);
				}

			}
		

		// Apply motor power
		elevatorDriverMain.set(ControlMode.PercentOutput, motorPower);
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Commanded arm height", commandedHeight);
		SmartDashboard.putNumber("Estimated arm height", estimatedHeight);
		SmartDashboard.putBoolean("Manual Override", manualOverride);
	}

	public void run() {
		while (true) {
			performMainProcessing();
			updateTelemetry();
		}
	}
}
