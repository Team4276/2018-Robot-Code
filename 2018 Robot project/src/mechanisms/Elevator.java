package mechanisms;

import org.usfirst.frc.team4276.robot.Robot;

import utilities.Xbox;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Elevator {

	final double POWER_OFFSET_MANIPULATOR = 4;
	final double POWER_OFFSET_RAIL = 7;
	final double KP_MANIPULATOR = 3;
	final double KP_RAIL = 5;
	final double MANIPULATOR_RAIL_HEIGHT = 10;
	final double RAIL_HEIGHT = 5;
	final double SETPOINT_SCALE = 7;
	final double SETPOINT_SWITCH = 3;
	final double SETPOINT_BOTTOM = 1;
	final double SETPOINT_INCREMENT = 5;

	double estimatedHeight = 0;
	double commandedHeight = 0;
	double heightError = 0;
	double motorPower;

	final double initiliazedPoint = 0;
	final double derivativePoint = 0;

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
		if (commandedHeight > MANIPULATOR_RAIL_HEIGHT){
			commandedHeight = MANIPULATOR_RAIL_HEIGHT;
		} else if(commandedHeight > SETPOINT_BOTTOM){
			commandedHeight = SETPOINT_BOTTOM;
		}

		if (commandedHeight < MANIPULATOR_RAIL_HEIGHT) {
			// insert estimatedHeight = encoder position
			heightError = commandedHeight - estimatedHeight;
			motorPower = POWER_OFFSET_MANIPULATOR + (KP_MANIPULATOR * heightError);
			// insert apply power to motors
		} else {
			// insert estimatedHeight = encoder position
			heightError = commandedHeight - estimatedHeight;
			motorPower = POWER_OFFSET_RAIL + (KP_RAIL * heightError);
			// insert apply power to motors
		}
		SmartDashboard.putNumber("Commanded arm height" ,commandedHeight);
		SmartDashboard.putNumber("Estimated arm height" ,estimatedHeight);
	}
}
