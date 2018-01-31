package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.Xbox;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

	TalonSRX elevatorDriverMain;
	VictorSPX elevatorDriverFollow;

	final double POWER_OFFSET_MANIPULATOR = 4;
	final double POWER_OFFSET_RAIL = 7;
	final double KP_MANIPULATOR = 3;
	final double KP_RAIL = 5;
	final double MANIPULATOR_RAIL_HEIGHT = 10;
	final double RAIL_HEIGHT = 5;
	final double SETPOINT_SCALE = 7;
	final double SETPOINT_SWITCH = 3;
	final double SETPOINT_BOTTOM = 1;
	final double SETPOINT_INCREMENT = .05;
	final double HEIGHT_PER_PULSE = 1;
	
	private boolean manualOveride;
	private boolean initializePID = true;
	private double timeNow;
	private double timePrevious;
	private double timeStep;
	private double accumulatedError; 
	private double rateError = 0;
	
	double estimatedHeight = 0;
	double commandedHeight = 0;
	double heightError = 0;
	double motorPower;
	double assignedPower;
	double heightErrorLast = 0;
	
	final double INITIALIZED_POINT = 0;
	final double DERIVATIVE_POINT = 0;

	public Elevator(int elevator1CANPort, int elevator2CANPort) {

		elevatorDriverMain = new TalonSRX(elevator1CANPort);
		elevatorDriverFollow = new VictorSPX(elevator2CANPort);

		elevatorDriverFollow.set(ControlMode.Follower, elevator1CANPort);

	}


	public void performMainProcess() {

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

/*		// Start + Right Axis Y (Manually control Height by Power)
		if (Robot.xboxController.getRawButton(Xbox.Start)
				&& (Math.abs(Robot.xboxController.getRawAxis(Xbox.RAxisY)) > -0.5)) {
			manualOveride = true;
		} else {
			manualOveride = false;
		}*/

		// Right Axis Y (Manually Change Setpoint)
		if (Robot.xboxController.getRawAxis(Xbox.RAxisY) > 0.5) {
			commandedHeight = commandedHeight + SETPOINT_INCREMENT;
		} else if (Robot.xboxController.getRawAxis(Xbox.RAxisY) < -0.5) {
			commandedHeight = commandedHeight - SETPOINT_INCREMENT;
		}

		// Limit commanded height range
		if (commandedHeight > MANIPULATOR_RAIL_HEIGHT) {
			commandedHeight = MANIPULATOR_RAIL_HEIGHT;
		} else if (commandedHeight > SETPOINT_BOTTOM) {
			commandedHeight = SETPOINT_BOTTOM;
		}
		
		if (initializePID == true) {
			timeNow = Robot.systemTimer.get();
			estimatedHeight = elevatorDriverMain.getSensorCollection().getQuadraturePosition() * HEIGHT_PER_PULSE;
			heightError = commandedHeight - estimatedHeight; 
			accumulatedError = 0.0;
			motorPower = 0.0;
			initializePID = false; 
		} else {
			heightErrorLast = heightError;
			timePrevious = timeNow;
			timeNow = Robot.systemTimer.get();
			estimatedHeight = elevatorDriverMain.getSensorCollection().getQuadraturePosition() * HEIGHT_PER_PULSE;	
			timeStep = timeNow - timePrevious;
			
			
			heightError = commandedHeight - estimatedHeight; 
			accumulatedError = accumulatedError + ((heightErrorLast + heightError) / 2) * timeStep;
			
			
			
			rateError = -elevatorDriverMain.getSensorCollection().getQuadratureVelocity() * HEIGHT_PER_PULSE * 10;
			
			//motorPower = p
			//This is where i left off
			
		}
		
		
	
		
		// PID control
		if (commandedHeight < MANIPULATOR_RAIL_HEIGHT) {
			// PID for manipulator rail
			estimatedHeight = elevatorDriverMain.getSensorCollection().getQuadraturePosition();
			heightError = commandedHeight - estimatedHeight;
			motorPower = POWER_OFFSET_MANIPULATOR + (KP_MANIPULATOR * heightError);
			elevatorDriverMain.set(ControlMode.PercentOutput, motorPower);
		} else {
			// PID for second rail
			estimatedHeight = elevatorDriverMain.getSensorCollection().getQuadraturePosition();
			heightError = commandedHeight - estimatedHeight;
			motorPower = POWER_OFFSET_RAIL + (KP_RAIL * heightError);
			elevatorDriverMain.set(ControlMode.PercentOutput, motorPower);
		}
/*
		if (manualOveride == true) {
			elevatorDriverMain.set(ControlMode.PercentOutput, Robot.xboxController.getRawAxis(Xbox.RAxisY));
		}*/
	}

	public void updateTelemetry() {
		SmartDashboard.putNumber("Commanded arm height", commandedHeight);
		SmartDashboard.putNumber("Estimated arm height", estimatedHeight);
	}
}
