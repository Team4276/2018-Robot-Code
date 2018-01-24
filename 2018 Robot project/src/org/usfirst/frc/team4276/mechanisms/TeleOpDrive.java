package org.usfirst.frc.team4276.mechanisms;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

public class TeleOpDrive {

	Joystick leftJoystick, rightJoystick;
	DoubleSolenoid gearShifter;

	VictorSPX leftMotor;
	VictorSPX rightMotor;
	VictorSPX leftMotorFollower1;
	VictorSPX rightMotorFollower1;
	VictorSPX leftMotorFollower2;
	VictorSPX rightMotorFollower2;

	public double leftDrivePower, rightDrivePower;
	public int highShifter = 4;
	public int lowShifter = 3;

	public TeleOpDrive(int shifterAPort, int shifterBPort, int leftCANPort, int rightCANPort, int leftCANPort1,
			int rightCANPort1, int leftCANPort2, int rightCANPort2) {

		leftMotor = new VictorSPX(leftCANPort);
		rightMotor = new VictorSPX(rightCANPort);
		leftMotorFollower1 = new VictorSPX(leftCANPort1);
		rightMotorFollower1 = new VictorSPX(rightCANPort1);
		leftMotorFollower2 = new VictorSPX(leftCANPort2);
		rightMotorFollower2 = new VictorSPX(rightCANPort2);

		leftMotorFollower1.set(ControlMode.Follower, leftCANPort);
		rightMotorFollower1.set(ControlMode.Follower, rightCANPort);
		leftMotorFollower2.set(ControlMode.Follower, leftCANPort);
		rightMotorFollower2.set(ControlMode.Follower, rightCANPort);

		gearShifter = new DoubleSolenoid(shifterAPort, shifterBPort);

	}

	public void getJoystickValues() {

		leftDrivePower = leftJoystick.getY();
		rightDrivePower = rightJoystick.getY();

	}

	public void checkForGearShift() {
		boolean shiftHi = rightJoystick.getRawButton(highShifter);
		boolean shiftLo = rightJoystick.getRawButton(lowShifter);
		boolean isHi = (gearShifter.get() == DoubleSolenoid.Value.kForward);
		boolean isLo = (gearShifter.get() == DoubleSolenoid.Value.kReverse);

		if (shiftHi) {
			if (isLo) {
				gearShifter.set(DoubleSolenoid.Value.kForward);
			}
		} else if (shiftLo) {
			if (isHi) {
				gearShifter.set(DoubleSolenoid.Value.kReverse);
			}
		}

	}

	public void setMotorSpeeds() {
		leftMotor.set(ControlMode.PercentOutput, leftDrivePower);
		rightMotor.set(ControlMode.PercentOutput, rightDrivePower);
	}

	public void performMainProcessing() {
		getJoystickValues();
		checkForGearShift();
		setMotorSpeeds();
	}
}
