package org.usfirst.frc.team4276.systems;

import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.Xbox;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
	VictorSPX climberMotor1, climberMotor2;
	final double MOTOR_POWER = 1;
	boolean climbInProgress = false;

	public Climber(int canPort1, int canPort2) {
		climberMotor1 = new VictorSPX(canPort1);
		climberMotor2 = new VictorSPX(canPort2);
	}

	public void performMainProcessing() {
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

	public void updateTelemetry() {
		SmartDashboard.putBoolean("climb in progress", climbInProgress);
	}
}