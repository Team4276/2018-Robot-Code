package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.SoftwareTimer;
import org.usfirst.frc.team4276.utilities.Xbox;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

public class Manipulator {
	final double TRIGGER_THRESHOLD = 0.5;
	final double AUTO_GRAB_DELAY = 0.2;
	final Value CLOSED = DoubleSolenoid.Value.kReverse;
	final Value OPEN = DoubleSolenoid.Value.kForward;

	VictorSPX intakeMotorLeft;
	VictorSPX intakeMotorRight;
	SoftwareTimer armTimer;
	DigitalInput limitSwitch;

	boolean autoGrabInit = true;
	Value manipulatorValue;
	boolean manipulatorIsOpen;
	final double INTAKE_SPEED = 1;
	final double OUTAKE_SPEED = -0.3;
	final double SHOOT_SPEED = -1;

	final int OFF = 0;
	final int IN = 1;
	final int OUT = 2;
	int intakeMode = OFF;

	public Manipulator(int canPort1, int canPort2, int dioPort1) {
		intakeMotorLeft = new VictorSPX(canPort1);
		intakeMotorRight = new VictorSPX(canPort2);
		limitSwitch = new DigitalInput(dioPort1);
		armTimer = new SoftwareTimer();
	}

	public void intake() {
		intakeMotorLeft.set(ControlMode.PercentOutput, INTAKE_SPEED);
		intakeMotorRight.set(ControlMode.PercentOutput, INTAKE_SPEED);
		intakeMode = IN;
	}

	public void outake() {
		intakeMotorLeft.set(ControlMode.PercentOutput, OUTAKE_SPEED);
		intakeMotorRight.set(ControlMode.PercentOutput, OUTAKE_SPEED);
		intakeMode = OUT;
	}

	public void shoot() {
		intakeMotorLeft.set(ControlMode.PercentOutput, SHOOT_SPEED);
		intakeMotorRight.set(ControlMode.PercentOutput, SHOOT_SPEED);
		intakeMode = OUT;
	}

	public void stop() {
		intakeMotorLeft.set(ControlMode.PercentOutput, 0);
		intakeMotorRight.set(ControlMode.PercentOutput, 0);
		intakeMode = OFF;
	}

	public boolean collect() {
		// if no cube is intaked
		if (!limitSwitch.get()) {
			intake();
			return false;
		} else {
			stop();
			return true;
		}
	}

	/*
	 * public void openManipulator() { solenoid.set(OPEN); }
	 * 
	 * public void closeManipulator() { solenoid.set(CLOSED); }
	 */
	public void performMainProcessing() {
		// get desired manipulator position based on right trigger toggler
		/*
		 * rtToggler.updateMechanismState(TRIGGER_THRESHOLD);
		 * manipulatorCommandedOpen = rtToggler.getMechanismState();
		 * 
		 * manipulatorValue = solenoid.get(); manipulatorIsOpen =
		 * (manipulatorValue == OPEN);
		 * 
		 * // if pressing left trigger and autoGrab should be initialized if
		 * (Robot.xboxController.getRawAxis(Xbox.LT) > TRIGGER_THRESHOLD &&
		 * autoGrabInit) { // set timer, grab cube, and end init
		 * solenoid.set(CLOSED); armTimer.setTimer(AUTO_GRAB_DELAY);
		 * autoGrabInit = false; } // if time is up and already initialized else
		 * if (armTimer.isExpired() && !autoGrabInit) { // sets arm and elevator
		 * setpoints and sets for reinitialization
		 * Robot.armPivoter.commandSetpoint(80); Robot.elevator.commandedHeight
		 * = Robot.elevator.SETPOINT_PREP; autoGrabInit = true; } // open or
		 * close solenoid depending on desired manipulator position else if
		 * (manipulatorCommandedOpen) { solenoid.set(OPEN); } else {
		 * solenoid.set(CLOSED); }
		 */

		if (Robot.xboxController.getRawButton(Xbox.LB)) {
			intake();
		} else if (Robot.xboxController.getRawButton(Xbox.RB)) {
			outake();
		} else if (Robot.xboxController.getRawAxis(Xbox.RT) > TRIGGER_THRESHOLD) {
			shoot();
		} else {
			stop();
		}
		updateTelemetry();
	}

	public void updateTelemetry() {
		String readOut;
		if (intakeMode == OFF) {
			readOut = "Off";
		} else if (intakeMode == IN) {
			readOut = "Intake";
		} else if (intakeMode == OUT) {
			readOut = "Outake";
		} else {
			readOut = "ERROR";
		}
		SmartDashboard.putString("Cube Arm Status", readOut);
		// SmartDashboard.putBoolean("manipulator open", manipulatorIsOpen);
	}
}
