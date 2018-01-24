
package org.usfirst.frc.team4276.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

import mechanisms.*;
import autonomous.*;
import static utilities.RoboRioPorts.*;

import org.usfirst.frc.team4276.autonomous.PositionFinder;
import org.usfirst.frc.team4276.mechanisms.ArmPivoter;
import org.usfirst.frc.team4276.mechanisms.Climber;
import org.usfirst.frc.team4276.mechanisms.Elevator;
import org.usfirst.frc.team4276.mechanisms.Manipulator;
import org.usfirst.frc.team4276.mechanisms.TeleOpDrive;

public class Robot extends SampleRobot {
	public static Timer systemTimer;
	public static Joystick logitechJoystickL;
	public static Joystick logitechJoystickR;
	public static Joystick xboxController;

	TeleOpDrive robotTankDrive;
	Climber robotClimber;
	Elevator robotElevator;
	Manipulator cubeManipulator;
	ArmPivoter robotArmPivoter;
	PositionFinder robotPositioningSystem;

	public Robot() {
		// Master timer
		systemTimer = new Timer();
		systemTimer.start();

		// Controllers
		logitechJoystickL = new Joystick(0);
		logitechJoystickR = new Joystick(1);
		xboxController = new Joystick(2);

		// Autonomous

		robotPositioningSystem = new PositionFinder(DIO_DRIVE_LEFT_A, DIO_DRIVE_LEFT_B, DIO_DRIVE_RIGHT_A,
				DIO_DRIVE_RIGHT_B);

		// Mechanisms

		robotTankDrive = new TeleOpDrive(DRIVE_DOUBLE_SOLENOID_FWD, DRIVE_DOUBLE_SOLENOID_REV, CAN_DRIVE_L1,
				CAN_DRIVE_R1, CAN_DRIVE_L2, CAN_DRIVE_R2, CAN_DRIVE_L3, CAN_DRIVE_R3);

		robotClimber = new Climber(CAN_CLIMBER1, CAN_CLIMBER2);

		robotElevator = new Elevator(CAN_RAIL_DRIVER1, CAN_RAIL_DRIVER2);
		robotArmPivoter = new ArmPivoter(CAN_ARM_PIVOT);
		cubeManipulator = new Manipulator(CUBE_MANIPULATOR);

		// Threads

		robotPositioningSystem.start();

	}

	public void robotInit() {

		robotArmPivoter.performMainProcessing(-90);
	}

	public void autonomous() {

	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {

			// robotTankDrive.performMainProcessing();
			// robotClimber.performMainProcessing();
			// robotArmPivoter.performMainProcessing();
			// cubeManipulator.performMainProcessing();

			Timer.delay(0.005);
		}
	}

	public void test() {
	}
}
