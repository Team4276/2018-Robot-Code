package org.usfirst.frc.team4276.autonomous;

import org.usfirst.frc.team4276.mechanisms.ADIS16448_IMU;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;

public class PositionFinder extends Thread implements Runnable {

	Encoder placeholderLeft, placeholderRight;
	static ADIS16448_IMU robotIMU;

	private double[] previousXY = new double[2];
	public double[] coordinates = new double[2];


	public PositionFinder(int encoder1A, int encoder1B, int encoder2A, int encoder2B) {

		placeholderLeft = new Encoder(encoder1A, encoder1B);
		placeholderRight = new Encoder(encoder2A, encoder2B);

		robotIMU = new ADIS16448_IMU();

	}

	public double getHeading() {
		double theta = -1 * robotIMU.getYaw();
		return theta;
	}

	private double getDeltaPosition() {
		double PL = placeholderLeft.getDistance();
		double PR = placeholderRight.getDistance();
		double deltaPosition = 0.5 * (PL + PR);
		return deltaPosition;
	}

	public double[] findDeltaCoords() {
		double deltaX = Math.cos(getHeading()) * getDeltaPosition();
		double deltaY = Math.sin(getHeading()) * getDeltaPosition();
		double[] deltas = new double[] { deltaX, deltaY };
		return deltas;
	}

	public double[] getCurrentLocation() {

		return coordinates;

	}

	public void setStartPoint(double XY[]) {

		previousXY = XY;

	}

	public void sendCoordinatesToNetworkTables() {

	}

	private void updateSmartDashboard() {
		SmartDashboard.putNumber("Heading", getHeading());
		SmartDashboard.putNumberArray("Robot Coordinates:", coordinates);
	}

	public void run() {

		while (true) {
			coordinates[0] = previousXY[0] + findDeltaCoords()[0];
			coordinates[1] = previousXY[1] + findDeltaCoords()[1];
			previousXY = coordinates;

		}
	}
}
