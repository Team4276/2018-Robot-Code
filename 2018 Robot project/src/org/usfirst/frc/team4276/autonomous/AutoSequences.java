package org.usfirst.frc.team4276.autonomous;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.systems.*;
import org.usfirst.frc.team4276.utilities.SoftwareTimer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSequences {

	DriveTrain driveTrain;
	Manipulator manipulator;
	Elevator elevator;
	ArmPivoter armPivoter;
	SoftwareTimer phaseTimer;

	// gamedata values
	public int switchValue = 0;
	public int scaleValue = 0;

	// AutoSequences error references
	private final int GET_START_POSITION_ERROR = 1;
	private final int SWITCH_DETERMINE_ERROR = 2;

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

	public int setRoutine = 0;

	public final int DEFAULT = 0;
	public final int MID_TO_LEFT_SWITCH = 1;
	public final int MID_TO_RIGHT_SWITCH = 2;
	public final int LEFT_SWITCH = 3;
	public final int RIGHT_SWITCH = 4;
	public final int LEFT_SCALE = 5;
	public final int RIGHT_SCALE = 6;

	// command return status
	boolean coordinateReached = false;
	boolean headingReached = false;

	// currentState enabled array
	static boolean[] stateEnabled = new boolean[EXIT + 1];

	public AutoSequences(DriveTrain driveImport, Manipulator armImport, Elevator elevatorImport,
			ArmPivoter armImport2) {
		driveTrain = driveImport;
		manipulator = armImport;
		elevator = elevatorImport;
		armPivoter = armImport2;
		phaseTimer = new SoftwareTimer();

	}

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

	public void updateGameData() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();

		if (gameData.charAt(0) == 'L') {
			switchValue = LEFT_SWITCH;

		} else if (gameData.charAt(0) == 'R') {

			switchValue = RIGHT_SWITCH;
		}

		if (gameData.charAt(1) == 'L') {
			scaleValue = LEFT_SCALE;

		} else if (gameData.charAt(1) == 'R') {
			scaleValue = RIGHT_SCALE;
		}
	}

	public void defineStateEnabledStatus() {
		if (setRoutine == DEFAULT) {
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
		} else if (setRoutine == MID_TO_LEFT_SWITCH || setRoutine == MID_TO_RIGHT_SWITCH) {
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
		} else if (setRoutine == LEFT_SWITCH || setRoutine == RIGHT_SWITCH) {
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
		} else if (setRoutine == LEFT_SCALE || setRoutine == RIGHT_SCALE) {
			// set all false
			stateEnabled[INIT] = true;
			stateEnabled[DRIVE_OFF_WALL] = false;
			stateEnabled[ROTATE_OFF_WALL] = false;
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

	}

	// Main State Variables
	boolean loop;
	boolean performStateEntry;

	public void loop() {
		switch (currentState) {
		case INIT:
			currentStateName = "Init";
			// State entry
			if (performStateEntry) {

				PositionFinder.setStartPoint(getStartPosition());
				driveTrain.setLoGear();
				manipulator.closeManipulator();
				phaseTimer.setTimer(0.2);
				performStateEntry = false;

			}

			// State processing

			updateGameData();

			// State exit
			if (phaseTimer.isExpired()) {
				performStateExit();
			}

			break;

		case DRIVE_OFF_WALL:
			currentStateName = "Drive Off Wall";
			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(4);
				performStateEntry = false;
			}

			// State processing
			coordinateReached = driveTrain.driveToCoordinate(FieldLocations.CENTER_DRIVE_OFF_WALL);

			// State exit
			if (coordinateReached || phaseTimer.isExpired()) {
				performStateExit();

			}
			break;

		case ROTATE_OFF_WALL:
			currentStateName = "Rotate Off Wall";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(2);
				performStateEntry = false;
			}
			// State processing
			if (switchValue == LEFT_SWITCH) {
				headingReached = driveTrain.rotateToCoordinate(FieldLocations.SWITCH_PREP_LEFT);

			} else if (switchValue == RIGHT_SWITCH) {
				headingReached = driveTrain.rotateToCoordinate(FieldLocations.SWITCH_PREP_RIGHT);

			} else {
				SmartDashboard.putNumber("Auto Error", SWITCH_DETERMINE_ERROR);
			}
			// State exit
			if (headingReached || phaseTimer.isExpired()) {
				performStateExit();
			}
			break;

		case DRIVE_TO_SCORE_1:
			currentStateName = "Drive to Scale Prep";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(4);
				performStateEntry = false;
			}
			// State processing
			if (switchValue == LEFT_SWITCH) {
				coordinateReached = driveTrain.driveToCoordinate(FieldLocations.SWITCH_PREP_LEFT);

			} else if (switchValue == RIGHT_SWITCH) {
				coordinateReached = driveTrain.driveToCoordinate(FieldLocations.SWITCH_PREP_RIGHT);

			} else {
				SmartDashboard.putNumber("Auto Error", SWITCH_DETERMINE_ERROR);
			}
			// State exit
			if (coordinateReached || phaseTimer.isExpired()) {
				performStateExit();
			}
			break;

		case ROTATE_TO_SCORE:
			currentStateName = "Rotate to Score";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(2);
				performStateEntry = false;
			}
			// State processing
			if (switchValue == LEFT_SWITCH) {
				headingReached = driveTrain.rotateToCoordinate(FieldLocations.SWITCH_PREP_LEFT);

			} else if (switchValue == RIGHT_SWITCH) {
				headingReached = driveTrain.rotateToCoordinate(FieldLocations.SWITCH_PREP_RIGHT);

			} else {
				SmartDashboard.putNumber("Auto Error", SWITCH_DETERMINE_ERROR);
			}
			// State exit
			if (headingReached || phaseTimer.isExpired()) {
				performStateExit();
			}
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
