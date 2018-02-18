package org.usfirst.frc.team4276.autonomous;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.SoftwareTimer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSequences {

	SoftwareTimer phaseTimer;
	// constants
	private double armDown = 0; // straight out

	// gamedata values
	public int switchValue = 0;
	public int scaleValue = 0;
	boolean startIsCenter = false;
	boolean startIsRight = false;
	boolean startIsLeft = false;

	// AutoSequences error references
	private final int GET_START_POSITION_ERROR = 1;
	private final int ROUTINE_DETERMINE_ERROR = 2;
	private final int ROUTE_PLAN_ERROR = 3;

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

	// routines
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

	public AutoSequences() {
		phaseTimer = new SoftwareTimer();

	}

	public double[] getStartPosition() {
		double[] startPosition = new double[] { 0, 0 }; // default
		if (RoutineSelector.startingPosition == RoutineSelector.CENTER) {
			startPosition = FieldLocations.CENTER_START_POSITION;
			startIsCenter = true;
			startIsRight = false;
			startIsLeft = false;
		} else if (RoutineSelector.startingPosition == RoutineSelector.LEFT) {
			startPosition = FieldLocations.LEFT_START_POSITION;
			startIsCenter = false;
			startIsRight = false;
			startIsLeft = true;
		} else if (RoutineSelector.startingPosition == RoutineSelector.RIGHT) {
			startPosition = FieldLocations.RIGHT_START_POSITION;
			startIsCenter = false;
			startIsRight = true;
			startIsLeft = false;
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

	public void planRoute() {
		getStartPosition();
		updateGameData();
		if (startIsCenter) {
			// if start in middle

			if (switchValue == LEFT_SWITCH) {
				// if switch to left
				setRoutine = MID_TO_LEFT_SWITCH;
			} else if (switchValue == RIGHT_SWITCH) {
				// if switch to right
				setRoutine = MID_TO_RIGHT_SWITCH;
			}
		} else if (startIsLeft) {
			// if start on left side

			if (switchValue == LEFT_SWITCH) {
				// if switch on this side
				setRoutine = LEFT_SWITCH;
			} else if (scaleValue == LEFT_SCALE) {
				// if scale on this side
				setRoutine = LEFT_SCALE;
			} else {
				setRoutine = DEFAULT;
			}
		} else if (startIsRight) {
			// if start on right side

			if (switchValue == RIGHT_SWITCH) {
				// if switch on this side
				setRoutine = RIGHT_SWITCH;
			} else if (scaleValue == RIGHT_SCALE) {
				// if scale on this side
				setRoutine = RIGHT_SCALE;
			} else {
				setRoutine = DEFAULT;
			}
		} else {
			SmartDashboard.putNumber("Auto Error", ROUTE_PLAN_ERROR);
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
		} else if (setRoutine == LEFT_SCALE || setRoutine == RIGHT_SCALE) {
			// set all false
			stateEnabled[INIT] = true;
			stateEnabled[DRIVE_OFF_WALL] = false;
			stateEnabled[ROTATE_OFF_WALL] = false;
			stateEnabled[DRIVE_TO_SCORE_1] = true;
			stateEnabled[ROTATE_TO_SCORE] = false;
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
				Robot.driveTrain.setLoGear();
				Robot.manipulator.closeManipulator();
				phaseTimer.setTimer(0.2);
				performStateEntry = false;

			}

			// State processing

			planRoute();
			defineStateEnabledStatus();
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
			if (setRoutine == DEFAULT) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.CROSS_BASE_LINE);
			} else {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.CENTER_DRIVE_OFF_WALL);
			}
			// State exit
			if (coordinateReached || phaseTimer.isExpired()) {
				Robot.driveTrain.resetDrive();
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
			if (setRoutine == MID_TO_LEFT_SWITCH) {
				headingReached = Robot.driveTrain.rotateToCoordinate(FieldLocations.SWITCH_PREP_LEFT_A);

			} else if (setRoutine == MID_TO_RIGHT_SWITCH) {
				headingReached = Robot.driveTrain.rotateToCoordinate(FieldLocations.SWITCH_PREP_RIGHT_A);

			} else {
				SmartDashboard.putNumber("Auto Error", ROUTINE_DETERMINE_ERROR);
			}
			// State exit
			if (headingReached || phaseTimer.isExpired()) {
				Robot.driveTrain.resetDrive();
				performStateExit();
			}
			break;

		case DRIVE_TO_SCORE_1:
			currentStateName = "Drive to Score Prep";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(4);
				performStateEntry = false;
			}
			// State processing
			if (setRoutine == MID_TO_LEFT_SWITCH) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.SWITCH_PREP_LEFT_A);

			} else if (setRoutine == MID_TO_RIGHT_SWITCH) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.SWITCH_PREP_RIGHT_A);

			} else if (setRoutine == LEFT_SWITCH) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.SWITCH_PREP_LEFT_B);

			} else if (setRoutine == RIGHT_SWITCH) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.SWITCH_PREP_RIGHT_B);

			} else if (setRoutine == RIGHT_SCALE) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.SWITCH_PREP_RIGHT_B);

			} else if (setRoutine == LEFT_SCALE) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.SWITCH_PREP_LEFT_B);

			} else {
				SmartDashboard.putNumber("Auto Error", ROUTINE_DETERMINE_ERROR);
			}
			// State exit
			if (coordinateReached || phaseTimer.isExpired()) {
				Robot.driveTrain.resetDrive();
				performStateExit();
			}
			break;

		case ROTATE_TO_SCORE:
			currentStateName = "Rotate to score";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(2);
				performStateEntry = false;
			}
			// State processing
			if (setRoutine == MID_TO_LEFT_SWITCH) {
				coordinateReached = Robot.driveTrain.rotateToCoordinate(FieldLocations.leftSwitchScoringZoneA);

			} else if (setRoutine == MID_TO_RIGHT_SWITCH) {
				coordinateReached = Robot.driveTrain.rotateToCoordinate(FieldLocations.rightSwitchScoringZoneA);

			} else if (setRoutine == LEFT_SWITCH) {
				coordinateReached = Robot.driveTrain.rotateToCoordinate(FieldLocations.leftSwitchScoringZoneB);

			} else if (setRoutine == RIGHT_SWITCH) {
				coordinateReached = Robot.driveTrain.rotateToCoordinate(FieldLocations.rightSwitchScoringZoneB);
			} else {
				SmartDashboard.putNumber("Auto Error", ROUTINE_DETERMINE_ERROR);
			}
			// State exit
			if (coordinateReached || phaseTimer.isExpired()) {
				Robot.driveTrain.resetDrive();
				performStateExit();
			}
			break;

		case RAISE_ELEVATOR:
			currentStateName = "Raise Elevator";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(.0001);
				performStateEntry = false;
			}
			// State processing
			if (setRoutine == MID_TO_LEFT_SWITCH || setRoutine == MID_TO_RIGHT_SWITCH || setRoutine == LEFT_SWITCH
					|| setRoutine == RIGHT_SWITCH) {
				// bring elevator to switch scoring height
				Robot.elevator.commandToSwitch();

			} else if (setRoutine == LEFT_SCALE || setRoutine == RIGHT_SCALE) {

				// bring elevator to scale scoring height
				Robot.elevator.commandToScale();

			} else {
				SmartDashboard.putNumber("Auto Error", ROUTINE_DETERMINE_ERROR);
			}
			// State exit
			if (phaseTimer.isExpired()) {
				performStateExit();
			}
			break;

		case LOWER_ARM:
			currentStateName = "Raise Elevator";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(.0001);
				performStateEntry = false;
			}
			// State processing

			Robot.armPivoter.commandSetpoint(armDown);
			// State exit
			if (phaseTimer.isExpired()) {
				performStateExit();
			}
			break;

		case DRIVE_TO_SCORE_2:
			currentStateName = "Drive to Score Deliver";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(4);
				performStateEntry = false;
			}
			// State processing
			if (setRoutine == MID_TO_LEFT_SWITCH) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.leftSwitchScoringZoneA);

			} else if (setRoutine == MID_TO_RIGHT_SWITCH) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.rightSwitchScoringZoneA);

			} else if (setRoutine == LEFT_SWITCH) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.leftSwitchScoringZoneB);

			} else if (setRoutine == RIGHT_SWITCH) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.rightSwitchScoringZoneB);

			} else if (setRoutine == RIGHT_SCALE) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.rightScaleScoringZoneA);

			} else if (setRoutine == LEFT_SCALE) {
				coordinateReached = Robot.driveTrain.driveToCoordinate(FieldLocations.leftScaleScoringZoneA);

			} else {
				SmartDashboard.putNumber("Auto Error", ROUTINE_DETERMINE_ERROR);
			}
			// State exit
			if (coordinateReached || phaseTimer.isExpired()) {
				Robot.driveTrain.resetDrive();
				performStateExit();
			}
			break;

		case RELEASE_CUBE:
			currentStateName = "Release Cube";
			// State entry
			if (performStateEntry) {

				Robot.manipulator.openManipulator();
				phaseTimer.setTimer(0.3);
				performStateEntry = false;
			}

			// State processing

			// State exit
			if (phaseTimer.isExpired()) {
				performStateExit();
			}

			break;

		case BACK_UP:
			currentStateName = "Back Up";
			// State entry
			if (performStateEntry) {

				phaseTimer.setTimer(2);
				performStateEntry = false;
			}
			// State processing
			Robot.driveTrain.reverse(true);
			// State exit
			if (phaseTimer.isExpired()) {
				Robot.driveTrain.reverse(false);
				Robot.driveTrain.resetDrive();
				performStateExit();
			}

			break;

		case LOWER_ELEVATOR:
			currentStateName = "Lower Elevator";

			// State entry
			if (performStateEntry) {
				phaseTimer.setTimer(.0001);
				performStateEntry = false;
			}
			// State processing
			Robot.elevator.commandToBottom();

			// State exit
			if (phaseTimer.isExpired()) {
				performStateExit();
			}
			break;

		case EXIT:
			Robot.driveTrain.resetDrive();
			currentStateName = "Autonomous Done";
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
