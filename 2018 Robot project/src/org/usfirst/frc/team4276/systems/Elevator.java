package org.usfirst.frc.team4276.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4276.robot.Robot;
import org.usfirst.frc.team4276.utilities.Xbox;
import org.usfirst.frc.team4276.utilities.Toggler;

public class Elevator extends Thread implements Runnable {

	private TalonSRX elevatorDriverMain;
	private VictorSPX elevatorDriverFollow;
	private Toggler manualOverrideToggler;

	// Constants - Lower Rail
	private double STATIC_GAIN_LOWER = 0.08;
	private double KP_LOWER = 750 * 1e-3;
	private double KI_LOWER = 1 * 1e-3;
	private double KD_LOWER = 8 * 1e-3;
	private final double MAX_HEIGHT_LOWER = 2.625; // ft

	// Constants - Upper Rail
	private double STATIC_GAIN_UPPER = STATIC_GAIN_LOWER;
	private double KP_UPPER = KP_LOWER;
	private double KI_UPPER = KI_LOWER;
	private double KD_UPPER = KD_LOWER;
	private final double MAX_HEIGHT_UPPER = 6.125; // ft

	// Constants - General
	private final double STARTING_HEIGHT = 1.7; // ft
	private final double SETPOINT_SCALE = 5.75; // ft
	private final double SETPOINT_SWITCH = 3; // ft
	private final double SETPOINT_BOTTOM = 0; // ft
	private final double SETPOINT_INCREMENT = .2; // ft
	private final double OVERRIDE_INCREMENT = 0.05; // 5%
	private final double HEIGHT_PER_PULSE = -1.562 * 1e-4; // 1/6400
	private final double MAX_POWER_UP = 0.7;
	private final double MAX_POWER_DOWN = 0.1;

	// General parameters
	private boolean manualOverrideIsEngaged;
	private double encoderOffset = 0;
	private double estimatedHeight = 0;
	private double commandedHeight = STARTING_HEIGHT;
	private double manualPower = 0;
	private double staticPower = 0;
	private double activePower = 0;
	private double commandedPower = 0;

	// PID parameters
	private boolean initializePID = true;
	private double heightError = 0;
	private double heightErrorLast = 0;
	private double accumulatedError = 0;
	private double rateError = 0;
	private double timeNow;
	private double timePrevious;
	private double timeStep;

	public Elevator(int elevator1CANPort, int elevator2CANPort) {
		elevatorDriverMain = new TalonSRX(elevator1CANPort);
		elevatorDriverFollow = new VictorSPX(elevator2CANPort);
		manualOverrideToggler = new Toggler(Xbox.Back);

		// elevatorDriverFollow.set(ControlMode.Follower, elevator1CANPort);
		encoderOffset = STARTING_HEIGHT
				- elevatorDriverMain.getSensorCollection().getQuadraturePosition() * HEIGHT_PER_PULSE;
	}

	private void computeManualPowerOffset() {
		if (Robot.xboxController.getRawAxis(Xbox.RAxisY) < -0.15) {
			manualPower = OVERRIDE_INCREMENT;
		} else if (Robot.xboxController.getRawAxis(Xbox.RAxisY) > 0.15) {
			manualPower = -OVERRIDE_INCREMENT;
		}
	}

	private void computeStaticPower() {
		if (estimatedHeight < MAX_HEIGHT_LOWER) {
			staticPower = STATIC_GAIN_LOWER;
		} else {
			staticPower = STATIC_GAIN_UPPER;
		}
	}

	private void determineSetpoint() {
		// button Y(Deposit Cube in Scale)
		if (Robot.xboxController.getRawButton(Xbox.Y)) {
			commandedHeight = SETPOINT_SCALE;
		}

		// button A(Deposit Cube in Switch)
		if (Robot.xboxController.getRawButton(Xbox.B)) {
			commandedHeight = SETPOINT_SWITCH;
		}

		// button B(Manipulator to Bottom)
		if (Robot.xboxController.getRawButton(Xbox.A)) {
			commandedHeight = SETPOINT_BOTTOM;
		}

		// Right Axis Y (Manually Change Setpoint)
		if (Robot.xboxController.getRawAxis(Xbox.RAxisY) < -0.15) {
			commandedHeight = commandedHeight + SETPOINT_INCREMENT;
		} else if (Robot.xboxController.getRawAxis(Xbox.RAxisY) > 0.15) {
			commandedHeight = commandedHeight - SETPOINT_INCREMENT;
		}

		// Limit commanded height range
		if (commandedHeight > MAX_HEIGHT_UPPER) {
			commandedHeight = MAX_HEIGHT_UPPER;
		} else if (commandedHeight < SETPOINT_BOTTOM) {
			commandedHeight = SETPOINT_BOTTOM;
		}
	}

	private void computeActivePower() {
		if (initializePID == true) {
			timeNow = Robot.systemTimer.get();
			estimatedHeight = elevatorDriverMain.getSensorCollection().getQuadraturePosition() * HEIGHT_PER_PULSE
					+ encoderOffset;
			heightError = commandedHeight - estimatedHeight;
			accumulatedError = 0.0;
			initializePID = false;
		} else {
			heightErrorLast = heightError;
			timePrevious = timeNow;
			timeNow = Robot.systemTimer.get();
			estimatedHeight = elevatorDriverMain.getSensorCollection().getQuadraturePosition() * HEIGHT_PER_PULSE
					+ encoderOffset;
			timeStep = timeNow - timePrevious;

			// Compute control errors
			heightError = commandedHeight - estimatedHeight; // height
			accumulatedError = accumulatedError + (heightErrorLast + heightError) / 2 * timeStep; // height*sec
			rateError = -elevatorDriverMain.getSensorCollection().getQuadratureVelocity() * 10 * HEIGHT_PER_PULSE; // height/sec

			// Compute PID active power
			if (estimatedHeight < MAX_HEIGHT_LOWER) {
				// PID for lower rail
				activePower = (KP_LOWER * heightError) + (KI_LOWER * accumulatedError) + (KD_LOWER * rateError);
			} else {
				// PID for upper rail
				activePower = (KP_UPPER * heightError) + (KI_UPPER * accumulatedError) + (KD_UPPER * rateError);
			}
		}
	}

	private void limitCommandedPower() {
		// Limit the range of commanded power
		if (commandedPower > MAX_POWER_UP) {
			commandedPower = MAX_POWER_UP;
		} else if (commandedPower < -MAX_POWER_DOWN) {
			commandedPower = -MAX_POWER_DOWN;
		}
	}

	private void tuneControlGains() {
		if (Robot.logitechJoystickL.getRawButton(5) == true) {
			STATIC_GAIN_LOWER = STATIC_GAIN_LOWER + 10e-3;
		}
		if (Robot.logitechJoystickL.getRawButton(3) == true) {
			STATIC_GAIN_LOWER = STATIC_GAIN_LOWER - 10e-3;
		}
		if (Robot.logitechJoystickL.getRawButton(7) == true) {
			KP_LOWER = KP_LOWER + 10e-3;
		}
		if (Robot.logitechJoystickL.getRawButton(8) == true) {
			KP_LOWER = KP_LOWER - 10e-3;
		}
		if (Robot.logitechJoystickL.getRawButton(9) == true) {
			KI_LOWER = KI_LOWER + 1e-3;
		}
		if (Robot.logitechJoystickL.getRawButton(10) == true) {
			KI_LOWER = KI_LOWER - 1e-3;
		}
		if (Robot.logitechJoystickL.getRawButton(11) == true) {
			KD_LOWER = KD_LOWER + 1e-3;
		}
		if (Robot.logitechJoystickL.getRawButton(12) == true) {
			KD_LOWER = KD_LOWER - 1e-3;
		}
		SmartDashboard.putNumber("Elevator Lower Kstatic", STATIC_GAIN_LOWER);
		SmartDashboard.putNumber("Elevator Lower Kp*1e-3", KP_LOWER * 1e3);
		SmartDashboard.putNumber("Elevator Lower Ki*1e-3", KI_LOWER * 1e3);
		SmartDashboard.putNumber("Elevator Lower Kd*1e-3", KD_LOWER * 1e3);

		if (Robot.logitechJoystickR.getRawButton(5) == true) {
			STATIC_GAIN_UPPER = STATIC_GAIN_UPPER + 10e-3;
		}
		if (Robot.logitechJoystickR.getRawButton(3) == true) {
			STATIC_GAIN_UPPER = STATIC_GAIN_UPPER - 10e-3;
		}
		if (Robot.logitechJoystickR.getRawButton(7) == true) {
			KP_UPPER = KP_UPPER + 10e-3;
		}
		if (Robot.logitechJoystickR.getRawButton(8) == true) {
			KP_UPPER = KP_UPPER - 10e-3;
		}
		if (Robot.logitechJoystickR.getRawButton(9) == true) {
			KI_UPPER = KI_UPPER + 1e-3;
		}
		if (Robot.logitechJoystickR.getRawButton(10) == true) {
			KI_UPPER = KI_UPPER - 1e-3;
		}
		if (Robot.logitechJoystickR.getRawButton(11) == true) {
			KD_UPPER = KD_UPPER + 1e-3;
		}
		if (Robot.logitechJoystickR.getRawButton(12) == true) {
			KD_UPPER = KD_UPPER - 1e-3;
		}
		SmartDashboard.putNumber("Elevator Upper Kstatic", STATIC_GAIN_UPPER);
		SmartDashboard.putNumber("Elevator Upper Kp*1e-3", KP_UPPER * 1e3);
		SmartDashboard.putNumber("Elevator Upper Ki*1e-3", KI_UPPER * 1e3);
		SmartDashboard.putNumber("Elevator Upper Kd*1e-3", KD_UPPER * 1e3);

		SmartDashboard.putNumber("Elevator power", commandedPower);
	}

	private void updateTelemetry() {
		SmartDashboard.putNumber("Commanded arm height", commandedHeight);
		SmartDashboard.putNumber("Estimated arm height", estimatedHeight);
		SmartDashboard.putBoolean("Elevator override", manualOverrideIsEngaged);
		SmartDashboard.putNumber("Elevator power", commandedPower);
	}

	public void commandToBottom() {
		commandedHeight = SETPOINT_BOTTOM;
	}

	public void commandToSwitch() {
		commandedHeight = SETPOINT_SWITCH;
	}

	public void commandToScale() {
		commandedHeight = SETPOINT_SCALE;
	}

	public void initializeThread() {
		commandedPower = 0;
		initializePID = true;
		accumulatedError = 0.0;
	}

	public void run() {
		while (true) {
			tuneControlGains(); // for gain tuning only - COMMENT THIS LINE OUT FOR
			// COMPETITION
			manualOverrideToggler.updateMechanismState();
			manualOverrideIsEngaged = manualOverrideToggler.getMechanismState();
			if (manualOverrideIsEngaged) {
				computeManualPowerOffset();
				commandedPower = commandedPower + manualPower;
			} else {
				determineSetpoint();
				computeStaticPower();
				computeActivePower();
				commandedPower = staticPower + activePower;
			}
			limitCommandedPower();
			elevatorDriverMain.set(ControlMode.PercentOutput, commandedPower);
			elevatorDriverFollow.set(ControlMode.PercentOutput, commandedPower);
			updateTelemetry();
			Timer.delay(0.125);
		}
	}
}
