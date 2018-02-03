package org.usfirst.frc.team4276.autonomous;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4276.systems.ADIS16448_IMU;

import edu.wpi.first.networktables.NetworkTable;

public class PositionFinder extends Thread implements Runnable {

	Encoder driveEncoderL, driveEncoderR;
	static ADIS16448_IMU robotIMU;

	static double[] previousXY = new double[2];
	public static double[] currentXY = new double[2];

	static double currentHeadingDeg = 0;
	static double currentHeadingRad = 0;

	static double currentX = 0;
	static double currentY = 0;

	public PositionFinder(int encoder1A, int encoder1B, int encoder2A, int encoder2B) {

		driveEncoderL = new Encoder(encoder1A, encoder1B);
		driveEncoderR = new Encoder(encoder2A, encoder2B);

		robotIMU = new ADIS16448_IMU();

	}

	private void updateHeading() {
		double currentHeadingTemp = -1 * robotIMU.getYaw();
		while (Math.abs(currentHeadingTemp) > 180) {
			if (currentHeadingTemp > 0) {
				currentHeadingTemp = currentHeadingTemp - 360;
			} else {
				currentHeadingTemp = currentHeadingTemp + 360;
			}
		}
		currentHeadingDeg = currentHeadingTemp;
		currentHeadingRad = Math.toRadians(currentHeadingDeg);
	}

	public static double getHeadingDeg() {
		return currentHeadingDeg;
	}

	public static double getHeadingRad() {
		return currentHeadingRad;
	}

	private void updatePosition() {
		double PL = driveEncoderL.getDistance();
		double PR = driveEncoderR.getDistance();
		driveEncoderL.reset();
		driveEncoderL.reset();
		double deltaPosition = 0.5 * (PL + PR);
		double deltaX = Math.cos(currentHeadingRad) * deltaPosition;
		double deltaY = Math.sin(currentHeadingRad) * deltaPosition;

		currentX = previousXY[0] + deltaX;
		currentY = previousXY[1] + deltaY;
		currentXY[0] = currentX;
		currentXY[1] = currentY;
	}

	public static double[] getCurrentLocation() {
		return currentXY;
	}

	public static void setStartPoint(double XY[]) {
		previousXY = XY;
	}

	private void updateSmartDashboard() {
		SmartDashboard.putNumber("Heading", currentHeadingDeg);
		SmartDashboard.putNumberArray("Robot Coordinates:", currentXY);
	}

	public void run() {

		while (true) {

			previousXY = currentXY;
			updateHeading();
			updatePosition();
			updateSmartDashboard();
			Timer.delay(00.125);
		}
	}
}
