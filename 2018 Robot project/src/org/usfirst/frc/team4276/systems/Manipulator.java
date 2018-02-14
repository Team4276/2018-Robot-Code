package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.utilities.Toggler;
import org.usfirst.frc.team4276.utilities.Xbox;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Manipulator {
	final double TRIGGER_THRESHOLD = 0.5;
	final Value CLOSED = DoubleSolenoid.Value.kReverse;
	final Value OPEN = DoubleSolenoid.Value.kForward;

	DoubleSolenoid solenoid;
	Toggler rtToggler;
	
	boolean manipulatorCommandedOpen;
	Value manipulatorValue;
	boolean manipulatorIsOpen;

	public Manipulator(int PCMPort1, int PCMPort2) {
		solenoid = new DoubleSolenoid(PCMPort1, PCMPort2);
		rtToggler = new Toggler(Xbox.RT);
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

		// open or close solenoid depending on desired manipulator position
		if (manipulatorCommandedOpen) {
			solenoid.set(OPEN);
		} else {
			solenoid.set(CLOSED);
		}
		updateTelemetry();
	}

	public void updateTelemetry() {
		manipulatorValue = solenoid.get();
		manipulatorIsOpen = (manipulatorValue == OPEN);
		SmartDashboard.putBoolean("manipulator open", manipulatorIsOpen);
	}
}
