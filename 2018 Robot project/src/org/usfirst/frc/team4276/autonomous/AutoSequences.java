package org.usfirst.frc.team4276.autonomous;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.systems.*;
import org.usfirst.frc.team4276.utilities.SoftwareTimer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSequences {

	private final int GET_START_POSITION_ERROR = 1;

	public double[] getStartPosition() {
		double[] startPosition = new double[] { 0, 0 }; // default
		if (RoutineSelector.startingPosition == RoutineSelector.CENTER) {
			startPosition = FieldLocations.CENTER_START_POSITION;
		} else if (RoutineSelector.startingPosition == RoutineSelector.LEFT) {
			startPosition = FieldLocations.LEFT_START_POSITION;
		} else if (RoutineSelector.startingPosition == RoutineSelector.RIGHT) {
			startPosition = FieldLocations.RIGHT_START_POSITION;
		} else {
			SmartDashboard.putNumber("Auto Error", GET_START_POSITION_ERROR);
		}

		return startPosition;
	}

	String gameData = DriverStation.getInstance().getGameSpecificMessage();
	/*
	 * if (gameData.charAt(0) == 'L') ; else ; if (gameData.charAt(1) == 'L') ;
	 * else ; if (gameData.charAt(2) == 'L') ; else ;
	 */

	// currentState Definitions
	static final int INIT = 0;
	static final int DRIVE_OFF_WALL = 1;
	static final int ROTATE_OFF_WALL = 2;
	static final int DRIVE_TO_SCORE_1 = 3;
	static final int ROTATE_TO_SCORE = 4;
	static final int RAISE_ELEVATOR = 5;
	static final int LOWER_ARM = 6;
	static final int DRIVE_TO_SCORE_2 = 7;
	static final int RELEASE_CUBE = 8;
	static final int BACK_UP = 9;
	static final int LOWER_ELEVATOR = 10;
	static final int EXIT = 11;

	static int currentState;
	String currentStateName;

	public final int SET_ROUTINE = 0;

	public final int DEFAULT = 0;
	public final int MID_TO_LEFT_SWITCH = 1;
	public final int MID_TO_RIGHT_SWITCH = 2;
	public final int LEFT_SWITCH = 3;
	public final int RIGHT_SWITCH = 4;
	public final int LEFT_SCALE = 5;
	public final int RIGHT_SCALE = 6;

	// currentState enabled array
	static boolean[] stateEnabled = new boolean[EXIT + 1];

	public void defineStateEnabledStatus() {
		if (SET_ROUTINE == DEFAULT) {
			// set all false
			stateEnabled[INIT] = true;
			stateEnabled[DRIVE_OFF_WALL] = true;
			stateEnabled[ROTATE_OFF_WALL] = false;
			stateEnabled[DRIVE_TO_SCORE_1] = false;
			stateEnabled[ROTATE_TO_SCORE] = false;
			stateEnabled[RAISE_ELEVATOR] = false;
			stateEnabled[LOWER_ARM] = false;
			stateEnabled[DRIVE_TO_SCORE_2] = false;
			stateEnabled[RELEASE_CUBE] = false;
			stateEnabled[BACK_UP] = false;
			stateEnabled[LOWER_ELEVATOR] = false;
			stateEnabled[EXIT] = true;
		} else if (SET_ROUTINE == MID_TO_LEFT_SWITCH || SET_ROUTINE == MID_TO_RIGHT_SWITCH) {
			stateEnabled[INIT] = true;
			stateEnabled[DRIVE_OFF_WALL] = true;
			stateEnabled[ROTATE_OFF_WALL] = true;
			stateEnabled[DRIVE_TO_SCORE_1] = true;
			stateEnabled[ROTATE_TO_SCORE] = true;
			stateEnabled[RAISE_ELEVATOR] = true;
			stateEnabled[LOWER_ARM] = true;
			stateEnabled[DRIVE_TO_SCORE_2] = true;
			stateEnabled[RELEASE_CUBE] = true;
			stateEnabled[BACK_UP] = true;
			stateEnabled[LOWER_ELEVATOR] = true;
			stateEnabled[EXIT] = true;
		} else if (SET_ROUTINE == LEFT_SWITCH) {
			// set all false
			stateEnabled[INIT] = true;
			stateEnabled[DRIVE_OFF_WALL] = true;
			stateEnabled[ROTATE_OFF_WALL] = false;
			stateEnabled[DRIVE_TO_SCORE_1] = false;
			stateEnabled[ROTATE_TO_SCORE] = true;
			stateEnabled[RAISE_ELEVATOR] = true;
			stateEnabled[LOWER_ARM] = true;
			stateEnabled[DRIVE_TO_SCORE_2] = true;
			stateEnabled[RELEASE_CUBE] = true;
			stateEnabled[BACK_UP] = true;
			stateEnabled[LOWER_ELEVATOR] = true;
			stateEnabled[EXIT] = true;
		}

		stateEnabled[INIT] = true;
		stateEnabled[DRIVE_OFF_WALL] = true;
		stateEnabled[ROTATE_OFF_WALL] = true;
		stateEnabled[DRIVE_TO_SCORE_1] = true;
		stateEnabled[ROTATE_TO_SCORE] = true;
		stateEnabled[RAISE_ELEVATOR] = true;
		stateEnabled[LOWER_ARM] = true;
		stateEnabled[DRIVE_TO_SCORE_2] = true;
		stateEnabled[RELEASE_CUBE] = true;
		stateEnabled[BACK_UP] = true;
		stateEnabled[LOWER_ELEVATOR] = true;
		stateEnabled[EXIT] = true;
	}

	// Main State Variables
	boolean loop;
	boolean performStateEntry;
	boolean performStateExit;

	public void loop() {
		switch (currentState) {
		case INIT:

			break;

		case DRIVE_OFF_WALL:

			break;

		case ROTATE_OFF_WALL:

			break;

		case DRIVE_TO_SCORE_1:

			break;

		case ROTATE_TO_SCORE:

			break;

		case RAISE_ELEVATOR:

			break;

		case LOWER_ARM:

			break;

		case DRIVE_TO_SCORE_2:

			break;

		case RELEASE_CUBE:

			break;

		case BACK_UP:

			break;

		case LOWER_ELEVATOR:

			break;

		case EXIT:

			break;

		}
	}

	private void performStateExit() {
		performStateEntry = true;
		performStateExit = false;
		advanceState();
	}

	public void advanceState() {
		if (stateEnabled[(currentState + 1)]) {
			currentState = currentState + 1;
		} else {
			currentState = currentState + 1;
			advanceState();
		}
	}
}
