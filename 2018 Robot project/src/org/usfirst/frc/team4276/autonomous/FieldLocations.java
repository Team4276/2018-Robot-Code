package org.usfirst.frc.team4276.autonomous;

public class FieldLocations {

	public static double[][] SCALE_LOCATION = new double[2][2];

	public static final double[] CENTER_START_POSITION = new double[] { 1.42, 13.5 };
	public static final double[] LEFT_START_POSITION = new double[] { 1.42, 22 };
	public static final double[] RIGHT_START_POSITION = new double[] { 1.42, 5 };
	
	public static final double[] CENTER_DRIVE_OFF_WALL = new double[] { 4.42, 13.5 };
	public static final double[] SWITCH_PREP_LEFT = new double[] { 7.2, 18.2 };
	public static final double[] SWITCH_PREP_RIGHT = new double[] { 7.2, 8.8 };

	public static final double[] leftScaleLocation = new double[] { 27, 19.5 };
	public static final double[] rightScaleLocation = new double[] { 27, 7.5 };
	public static final double[] leftSwitchLocation = new double[] { 14, 18 };
	public static final double[] rightSwitchLocation = new double[] { 14, 9 };
	public static final double[] leftCornerCubeLocation = new double[] { 16.9, 19.3 };
	public static final double[] rightCornerCubeLocation = new double[] { 16.9, 7.6 };

	public static final double[] leftScaleScoringZoneA = new double[] { 24.3, 19.9 };
	public static final double[] rightScaleScoringZoneA = new double[] { 24.3, 7 };
	public static final double[] leftSwitchScoringZoneA = new double[] { 11, 18.2 };
	public static final double[] rightSwitchScoringZoneA = new double[] { 11, 8.8 };
	public static final double[] leftScaleScoringZoneB = new double[] { 27, 21.7 };
	public static final double[] rightScaleScoringZoneB = new double[] { 27, 5.3 };
	public static final double[] leftSwitchScoringZoneB = new double[] { 13.9, 20.6 };
	public static final double[] rightSwitchScoringZoneB = new double[] { 13.9, 6.4 };
	public static final int LEFT = 0;
	public static final int RIGHT = 1;
	public static final int X = 0;
	public static final int Y = 1;

	public void inititalize() {
		SCALE_LOCATION[LEFT][X] = 27;
		SCALE_LOCATION[LEFT][Y] = 19.5;

		// SCALE_LOCATION[RT][X] = 27;
		// SCALE_LOCATION[LEFT][Y] = 19.5;
	}

}
