
package org.usfirst.frc.team4276.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team4276.autonomous.PositionFinder;
import org.usfirst.frc.team4276.autonomous.AutoSequences;

import org.usfirst.frc.team4276.systems.ArmPivoter;
import org.usfirst.frc.team4276.systems.Climber;
import org.usfirst.frc.team4276.systems.DriveTrain;
import org.usfirst.frc.team4276.systems.Elevator;
import org.usfirst.frc.team4276.systems.Manipulator;
import org.usfirst.frc.team4276.systems.Cameras;

import org.usfirst.frc.team4276.utilities.RoboRioPorts;

public class Robot extends SampleRobot {
	public static Timer systemTimer;
	public static Joystick logitechJoystickL;
	public static Joystick logitechJoystickR;
	public static Joystick xboxController;

	public static Cameras robotCameraSystem;
	public static DriveTrain driveTrain;
	public static Climber climber;
	public static Elevator elevator;
	public static Manipulator manipulator;
	public static ArmPivoter armPivoter;

	public static PositionFinder positionFinder;
	public AutoSequences autoSequences;

	public Robot() {
		// Master timer
		systemTimer = new Timer();
		systemTimer.start();

		// Controllers
		logitechJoystickL = new Joystick(0);
		logitechJoystickR = new Joystick(1);
		xboxController = new Joystick(2);

		// Mechanisms

		robotCameraSystem = new Cameras();

		driveTrain = new DriveTrain(RoboRioPorts.DRIVE_DOUBLE_SOLENOID_FWD, RoboRioPorts.DRIVE_DOUBLE_SOLENOID_REV,
				RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_L2,
				RoboRioPorts.CAN_DRIVE_R2, RoboRioPorts.CAN_DRIVE_L3, RoboRioPorts.CAN_DRIVE_R3);

		climber = new Climber(RoboRioPorts.CAN_CLIMBER1, RoboRioPorts.CAN_CLIMBER2, RoboRioPorts.CLIMBER_PISTON);

		elevator = new Elevator(RoboRioPorts.CAN_RAIL_DRIVER1, RoboRioPorts.CAN_RAIL_DRIVER2);

		armPivoter = new ArmPivoter(RoboRioPorts.CAN_ARM_PIVOT);

		manipulator = new Manipulator(RoboRioPorts.GRABBER_PISTON_FWD, RoboRioPorts.GRABBER_PISTON_REV);

		// Autonomous

		positionFinder = new PositionFinder(RoboRioPorts.DIO_DRIVE_LEFT_A, RoboRioPorts.DIO_DRIVE_LEFT_B,
				RoboRioPorts.DIO_DRIVE_RIGHT_A, RoboRioPorts.DIO_DRIVE_RIGHT_B);
		autoSequences = new AutoSequences();

		// Threads

		armPivoter.start();
		
		elevator.start();
		
		positionFinder.start();

	}

	public void robotInit() {

	}

	public void autonomous() {
		while (isEnabled()) {
			autoSequences.loop();
			Timer.delay(0.05);
		}

	}

	public void operatorControl() {

		positionFinder.kill();

		while (isOperatorControl() && isEnabled()) {

			driveTrain.performMainProcessing();
			// robotClimber.performMainProcessing();
			// robotArmPivoter.performMainProcessing();
			// cubeManipulator.performMainProcessing();

			Timer.delay(0.05);
		}
	}

	public void test() {
	}
}
