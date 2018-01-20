package mechanisms;

import org.usfirst.frc.team4276.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import utilities.Xbox;

public class Climber {
	VictorSPX climberMotor;
	final double MOTOR_POWER = 1;

	public Climber(int canPort) {
		climberMotor = new VictorSPX(canPort);

	}

	public void performMainProcessing() {
		if (Robot.xboxController.getPOV(Xbox.DPad) == Xbox.POVup) {
			climberMotor.set(ControlMode.PercentOutput, MOTOR_POWER);

		} else {
			climberMotor.set(ControlMode.PercentOutput, 0);
		}

	}

}
