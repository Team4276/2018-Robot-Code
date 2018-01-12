package autonomous;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.NetworkTable;

public class robotPositionFinder extends Thread implements Runnable {

	Encoder placeholderLeft, placeholderRight;
	double[] previousXY = new double[2];
	double[] coordinates = new double[2];

	public robotPositionFinder(int a, int b, int c, int d) {

		placeholderLeft = new Encoder(a, b);
		placeholderRight = new Encoder(c, d);

	}

	private double getHeading() {
		double theta = 0;// place holder code
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
	
	
	public void sendCoordinatesToNetworkTables(){
		
	}

	public void run() {

		while (true) {
			coordinates[0] = previousXY[0] + findDeltaCoords()[0];
			coordinates[1] = previousXY[1] + findDeltaCoords()[1];
			previousXY[0] = coordinates[0];
			previousXY[1] = coordinates[1];
		}
	}
}
