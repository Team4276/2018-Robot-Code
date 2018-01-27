package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.utilities.Toggler;
import org.usfirst.frc.team4276.utilities.Xbox;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Manipulator {
	Solenoid solenoid;
	Toggler rtToggler;
	final double TRIGGER_THRESHOLD = -0.5;
	boolean manipulatorCommandedOpen;
	boolean solenoidIsOpen;

	public Manipulator(int canPort) {
		solenoid = new Solenoid(canPort);
		rtToggler = new Toggler(Xbox.RT);
	}

	public void openManipulator() {
		// get current solenoid status
		solenoidIsOpen = solenoid.get();

		// if solenoid is not(!) open, then open it
		if (!solenoidIsOpen) {
			solenoid.set(true);
		}
	}

	public void closeManipulator() {
		// get current solenoid status
		solenoidIsOpen = solenoid.get();

		// if solenoid is open, then close it
		if (solenoidIsOpen) {
			solenoid.set(false);
		}
	}

	public void performMainProcessing() {
		// get desired manipulator position based on right trigger toggler
		rtToggler.updateMechanismState(TRIGGER_THRESHOLD);
		manipulatorCommandedOpen = rtToggler.getMechanismState();

		// open or close solenoid depending on desired manipulator position
		solenoid.set(manipulatorCommandedOpen);
	}

	public void updateTelemetry() {
		solenoidIsOpen = solenoid.get();
		SmartDashboard.putBoolean("manipulator open", solenoidIsOpen);
	}
}
