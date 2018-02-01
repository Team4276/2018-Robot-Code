package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.Xbox;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
	Solenoid releaseTrigger;
	VictorSPX climberMotor1, climberMotor2;
	final double MOTOR_POWER = 1;
	boolean climbInProgress = false;

	public Climber(int canPort1, int canPort2, int PnuematicPort) {
		climberMotor1 = new VictorSPX(canPort1);
		climberMotor2 = new VictorSPX(canPort2);
		releaseTrigger = new Solenoid(PnuematicPort);
	}

	public void performMainProcessing() {
		checkForNuclearKeyTurn();
		if (Robot.xboxController.getPOV(Xbox.DPad) == Xbox.POVup) {
			climberMotor1.set(ControlMode.PercentOutput, MOTOR_POWER);
			climberMotor2.set(ControlMode.PercentOutput, MOTOR_POWER);
			// when climber is on climbInProgress = true (bookkept)
			climbInProgress = true;

		} else {
			climberMotor1.set(ControlMode.PercentOutput, 0);
			climberMotor2.set(ControlMode.PercentOutput, 0);
			// when climber is off climbInProgress = false (bookkept)
			climbInProgress = false;
		}
	}

	public void checkForNuclearKeyTurn() {
		if (Robot.logitechJoystickR.getRawButton(1) && Robot.xboxController.getRawAxis(Xbox.LT) > .05) {
			releaseTrigger.set(true);
		}
	}

	public void updateTelemetry() {
		SmartDashboard.putBoolean("climb in progress", climbInProgress);
	}
}