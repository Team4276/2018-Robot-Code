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

	public void performMainProcessing() {

		rtToggler.updateMechanismState(TRIGGER_THRESHOLD);

		manipulatorCommandedOpen = rtToggler.getMechanismState();

		solenoid.set(manipulatorCommandedOpen);

		solenoidIsOpen = solenoid.get();

		SmartDashboard.putBoolean("manipulator open", solenoidIsOpen);

	}

}
