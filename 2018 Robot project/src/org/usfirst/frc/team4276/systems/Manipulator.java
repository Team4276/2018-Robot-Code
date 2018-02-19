package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.SoftwareTimer;
import org.usfirst.frc.team4276.utilities.Toggler;
import org.usfirst.frc.team4276.utilities.Xbox;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Manipulator {
	final double TRIGGER_THRESHOLD = 0.5;
	final double AUTO_GRAB_DELAY = 0.2;
	final Value CLOSED = DoubleSolenoid.Value.kReverse;
	final Value OPEN = DoubleSolenoid.Value.kForward;

	DoubleSolenoid solenoid;
	Toggler rtToggler;
	SoftwareTimer armTimer;

	boolean autoGrabInit = true;
	boolean manipulatorCommandedOpen;
	Value manipulatorValue;
	boolean manipulatorIsOpen;

	public Manipulator(int PCMPort1, int PCMPort2) {
		solenoid = new DoubleSolenoid(PCMPort1, PCMPort2);
		rtToggler = new Toggler(Xbox.RT);
		armTimer = new SoftwareTimer();
	}

	public void openManipulator() {
		solenoid.set(OPEN);
	}

	public void closeManipulator() {
		solenoid.set(CLOSED);
	}

	public void performMainProcessing() {
		// get desired manipulator position based on right trigger toggler
		rtToggler.updateMechanismState(TRIGGER_THRESHOLD);
		manipulatorCommandedOpen = rtToggler.getMechanismState();

		manipulatorValue = solenoid.get();
		manipulatorIsOpen = (manipulatorValue == OPEN);

		// if pressing left trigger and autoGrab should be initialized
		if (Robot.xboxController.getRawAxis(Xbox.LT) > TRIGGER_THRESHOLD && autoGrabInit) {
			// set timer, grab cube, and end init
			solenoid.set(CLOSED);
			armTimer.setTimer(AUTO_GRAB_DELAY);
			autoGrabInit = false;
		}
		// if time is up and already initialized
		else if (armTimer.isExpired() && !autoGrabInit) {
			// sets arm and elevator setpoints and sets for reinitialization
			Robot.armPivoter.commandSetpoint(80);
			Robot.elevator.commandedHeight = Robot.elevator.SETPOINT_PREP;
			autoGrabInit = true;
		}
		// open or close solenoid depending on desired manipulator position
		else if (manipulatorCommandedOpen) {
			solenoid.set(OPEN);
		} else {
			solenoid.set(CLOSED);
		}
		updateTelemetry();
	}

	public void updateTelemetry() {
		SmartDashboard.putBoolean("manipulator open", manipulatorIsOpen);
	}
}
