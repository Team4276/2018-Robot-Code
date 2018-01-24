package org.usfirst.frc.team4276.mechanisms;

import java.awt.Robot;

import org.usfirst.frc.team4276.utilities.Xbox;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

	TalonSRX elevatorDriver1, elevatorDriver2;

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

	double estimatedHeight = 0;
	double commandedHeight = 0;
	double heightError = 0;
	double motorPower;

	final double initiliazedPoint = 0;
	final double derivativePoint = 0;

	public Elevator(int elevator1CANPort, int elevator2CANPort) {

		elevatorDriver1 = new TalonSRX(elevator1CANPort);
		elevatorDriver2 = new TalonSRX(elevator2CANPort);

		elevatorDriver2.set(ControlMode.Follower, elevator2CANPort);

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
		if (Robot.xboxController.getRawAxis(Xbox.RAxisY) > 0.5) {
			commandedHeight = commandedHeight + SETPOINT_INCREMENT;
		} else if (Robot.xboxController.getRawAxis(Xbox.RAxisY) < -0.5) {
			commandedHeight = commandedHeight - SETPOINT_INCREMENT;
		}
		if (commandedHeight > MANIPULATOR_RAIL_HEIGHT) {
			commandedHeight = MANIPULATOR_RAIL_HEIGHT;
		} else if (commandedHeight > SETPOINT_BOTTOM) {
			commandedHeight = SETPOINT_BOTTOM;
		}

		if (commandedHeight < MANIPULATOR_RAIL_HEIGHT) {
			estimatedHeight = elevatorDriver1.getSensorCollection().getQuadraturePosition();
			heightError = commandedHeight - estimatedHeight;
			motorPower = POWER_OFFSET_MANIPULATOR + (KP_MANIPULATOR * heightError);
			elevatorDriver1.set(ControlMode.PercentOutput, motorPower);
		} else {
			estimatedHeight = elevatorDriver1.getSensorCollection().getQuadraturePosition();
			heightError = commandedHeight - estimatedHeight;
			motorPower = POWER_OFFSET_RAIL + (KP_RAIL * heightError);
			elevatorDriver1.set(ControlMode.PercentOutput, motorPower);
		}
		SmartDashboard.putNumber("Commanded arm height", commandedHeight);
		SmartDashboard.putNumber("Estimated arm height", estimatedHeight);
	}
}
