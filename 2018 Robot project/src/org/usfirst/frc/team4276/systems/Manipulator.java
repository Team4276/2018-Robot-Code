package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.utilities.Toggler;
import org.usfirst.frc.team4276.utilities.Xbox;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Manipulator {
	DoubleSolenoid grabberSolenoid;
	Toggler rtToggler;
	final double TRIGGER_THRESHOLD = -0.5;
	boolean manipulatorCommandedOpen;

	final DoubleSolenoid.Value CLOSED = DoubleSolenoid.Value.kReverse;
	final DoubleSolenoid.Value OPEN = DoubleSolenoid.Value.kForward;
	DoubleSolenoid.Value currentState = null;

	public Manipulator(int PCMPort1, int PCMPort2) {
		grabberSolenoid = new DoubleSolenoid(PCMPort1, PCMPort2);
		rtToggler = new Toggler(Xbox.RT);
	}

	public void openManipulator() {
		currentState = (OPEN);
		updateMechanismState();
	}

	public void closeManipulator() {
		currentState = (CLOSED);
		updateMechanismState();
	}

	public void performMainProcessing() {
		// get desired manipulator position based on right trigger toggler
		rtToggler.updateMechanismState(TRIGGER_THRESHOLD);
		manipulatorCommandedOpen = rtToggler.getMechanismState();

		// open or close solenoid depending on desired manipulator position
		if (manipulatorCommandedOpen) {
			currentState = OPEN;
		} else {
			currentState = CLOSED;
		}
		updateMechanismState();
	}

	public void updateMechanismState() {
		grabberSolenoid.set(currentState);
	}

	public void updateTelemetry() {
		SmartDashboard.putBoolean("manipulator open", manipulatorCommandedOpen);
	}
}
