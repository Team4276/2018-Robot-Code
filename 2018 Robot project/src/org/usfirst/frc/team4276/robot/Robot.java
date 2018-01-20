
package org.usfirst.frc.team4276.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

import mechanisms.*;
import static utilities.RoboRioPorts.*;

public class Robot extends SampleRobot {
	public static Timer systemTimer;
	public static Joystick logitechJoystickL;
	public static Joystick logitechJoystickR;
	public static Joystick xboxController;

	TeleOpDrive robotTankDrive;

	public Robot() {
		// Master timer
		systemTimer = new Timer();
		systemTimer.start();

		// Controllers
		logitechJoystickL = new Joystick(0);
		logitechJoystickR = new Joystick(1);
		xboxController = new Joystick(2);

		// Autonomous

		// Mechanisms

		robotTankDrive = new TeleOpDrive(DRIVE_DOUBLE_SOLENOID_FWD, DRIVE_DOUBLE_SOLENOID_REV, CAN_DRIVE_L1,
				CAN_DRIVE_R1, CAN_DRIVE_L2, CAN_DRIVE_R2, CAN_DRIVE_L3, CAN_DRIVE_R3);
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
