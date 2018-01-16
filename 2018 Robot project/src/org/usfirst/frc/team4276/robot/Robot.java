
package org.usfirst.frc.team4276.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import autonomous.robotPositionFinder;

public class Robot extends SampleRobot {
	public static Timer systemTimer;
	public static Joystick logitechJoystickL;
	public static Joystick logitechJoystickR;
	public static Joystick xboxController;

	robotPositionFinder rpf;

	public Robot() {
		// Master timer
		systemTimer = new Timer();
		systemTimer.start();

		// Controllers
		logitechJoystickL = new Joystick(0);
		logitechJoystickR = new Joystick(1);
		xboxController = new Joystick(2);

		// Autonomous
		rpf = new robotPositionFinder(0, 1, 2, 3);// placeholders TEST
		rpf.start();

		// Mechanisms
	}

	public void robotInit() {

	}

	public void autonomous() {

	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {

			Timer.delay(0.005);
		}
	}

	public void test() {
	}
}
