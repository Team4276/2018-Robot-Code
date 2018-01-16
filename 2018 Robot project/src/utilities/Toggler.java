package utilities;

import org.usfirst.frc.team4276.robot.Robot;

public class Toggler {

	private int button;
	private boolean state = false;

	private boolean currentButtonStatus = false;
	private boolean previousButtonStatus;

	public Toggler(int joystickButton) {
		button = joystickButton;
	}

	void updateMechanismState(double triggerValue) {
		previousButtonStatus = currentButtonStatus;
		currentButtonStatus = (Robot.xboxController.getRawAxis(button) >= triggerValue);
		if (currentButtonStatus == true) {
			if (previousButtonStatus == false) {
				if (state == true) {
					state = false;
				} else {
					state = true;
				}
			}
		}

	}

	void updateMechanismState(int DpadState) {
		previousButtonStatus = currentButtonStatus;
		currentButtonStatus = (DpadState == Robot.xboxController.getPOV(button));
		if (currentButtonStatus == true) {
			if (previousButtonStatus == false) {
				if (state == true) {
					state = false;
				} else {
					state = true;
				}
			}
		}
	}

	void updateMechanismState() {
		previousButtonStatus = currentButtonStatus;
		currentButtonStatus = Robot.xboxController.getRawButton(button);

		if (currentButtonStatus == true) {
			if (previousButtonStatus == false) {
				if (state == true) {
					state = false;
				} else {
					state = true;
				}
			}
		}
	}

	boolean getMechanismState() {
		return state;
	}
}
