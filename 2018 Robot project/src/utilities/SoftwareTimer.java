package utilities;

import org.usfirst.frc.team4276.robot.Robot;

public class SoftwareTimer {

	private double expirationTime;

	void setTimer(double timerValue) {
		expirationTime = Robot.systemTimer.get() + timerValue;
	}

	boolean isExpired() {
		return (Robot.systemTimer.get() > expirationTime);
		// if robotTime exceeds expirationTime, then this returns true
	}
}
