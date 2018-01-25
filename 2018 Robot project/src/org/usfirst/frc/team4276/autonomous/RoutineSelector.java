package org.usfirst.frc.team4276.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4276.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RoutineSelector {

	SendableChooser<Integer> selectionModeChooser;
	SendableChooser<Integer> startPosition;
	SendableChooser<Integer> strategyChooser;

	// Selection modes
	private final int COMMIT_MODE = 0;
	private final int EDIT_MODE = 1;
	private String[] selectionModeArray = new String[2];

	// Starting positions
	private final int LEFT = 0;
	private final int CENTER = 1;
	private final int RIGHT = 2;
	private String[] startPositionArray = new String[3];

	// Starting positions
	private final int CROSS_BASE = 0;
	private final int SCORE_SWITCH = 1;
	private final int SCORE_SCALE = 2;
	private String[] strategyArray = new String[3];

	private int selectionMode = COMMIT_MODE;
	static int startingPosition = 0;
	static int strategy = 0;
	static int autoModeToExecute;

	public RoutineSelector() {
		selectionModeArray[COMMIT_MODE] = "Commit mode";
		selectionModeArray[EDIT_MODE] = "Edit mode";

		startPositionArray[LEFT] = "left";
		startPositionArray[CENTER] = "center";
		startPositionArray[RIGHT] = "right";

		strategyArray[CROSS_BASE] = "cross base";
		strategyArray[SCORE_SWITCH] = "score switch";
		strategyArray[SCORE_SCALE] = "score scale";

		selectionModeChooser = new SendableChooser<Integer>();
		selectionModeChooser.addDefault(selectionModeArray[COMMIT_MODE], COMMIT_MODE);
		selectionModeChooser.addObject(selectionModeArray[EDIT_MODE], EDIT_MODE);
		SmartDashboard.putData("Commit Selections", selectionModeChooser);

		startPosition = new SendableChooser<Integer>();
		startPosition.addDefault(startPositionArray[CENTER], CENTER);
		startPosition.addObject(startPositionArray[LEFT], LEFT);
		startPosition.addObject(startPositionArray[RIGHT], RIGHT);
		SmartDashboard.putData("Starting Position", startPosition);

		strategyChooser = new SendableChooser<Integer>();
		strategyChooser.addDefault(startPositionArray[CROSS_BASE], CROSS_BASE);
		strategyChooser.addObject(startPositionArray[SCORE_SWITCH], SCORE_SWITCH);
		strategyChooser.addObject(startPositionArray[SCORE_SCALE], SCORE_SCALE);
		SmartDashboard.putData("Selected Strategy", strategyChooser);

	}
	
	public void run() {
		try{
		while (true) {
			if (selectionMode == COMMIT_MODE) {
				selectionMode = (int) selectionModeChooser.getSelected();
			} else {
				selectionMode = (int) selectionModeChooser.getSelected();
				startingPosition = (int) startPosition.getSelected();
				strategy = (int) strategyChooser.getSelected();
				autoModeToExecute = strategy + startingPosition;
			}
			SmartDashboard.putString("Selection mode", selectionModeArray[selectionMode]);
			SmartDashboard.putString("Alliance color", startPositionArray[startingPosition]);
			SmartDashboard.putString("Auto mode", strategyArray[strategy]);
			SmartDashboard.putNumber("Auto", autoModeToExecute);
			
			Robot.systemTimer.delay(0.1);
			SmartDashboard.putBoolean("AutoSelector Error", false);
		}
		}
		catch(Exception autoSelectorError)
		{

			SmartDashboard.putBoolean("AutoSelector Error", true);
			SmartDashboard.putString("Auto ERROR", autoSelectorError.getMessage());
		}
		
	}
}
