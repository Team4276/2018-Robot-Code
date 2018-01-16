package mechanisms;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
public class TeleOpDrive {

	Joystick leftJoystick, rightJoystick;
	DoubleSolenoid gearShifter;
	
	VictorSPX leftMotor;
	VictorSPX rightMotor;
	
	public double leftDrivePower, rightDrivePower;
	public int highShifter = 4;
	public int lowShifter = 3;

	public TeleOpDrive(int leftJoyPort, int rightJoyPort, int shifterAPort, int shifterBPort, int leftCANPort) {

		leftJoystick = new Joystick(leftJoyPort);
		rightJoystick = new Joystick(rightJoyPort);
		
		leftMotor = new VictorSPX(leftCANPort);

		gearShifter = new DoubleSolenoid(shifterAPort, shifterBPort);

	}
	
	public void getJoystickValues(){
		
		leftDrivePower = leftJoystick.getY();
		rightDrivePower = rightJoystick.getY();
		
	}
	
	public void checkForGearShift(){
		boolean shiftHi = rightJoystick.getRawButton(highShifter);
		boolean shiftLo = rightJoystick.getRawButton(lowShifter);
		boolean isHi = (gearShifter.get() == DoubleSolenoid.Value.kForward);
		boolean isLo = (gearShifter.get() == DoubleSolenoid.Value.kReverse);
		
		if(shiftHi){
			if(isLo){
				gearShifter.set(DoubleSolenoid.Value.kForward);
			}
		} else if(shiftLo){
			if(isHi){
				gearShifter.set(DoubleSolenoid.Value.kReverse);
			}
		}
		
	}
	public void setMotorSpeeds(){
		leftMotor.set(ControlMode.Velocity, leftDrivePower);
		rightMotor.set(ControlMode.Velocity, rightDrivePower);
	}
	
	public void performMainProcessing(){
		getJoystickValues();
		checkForGearShift();
		setMotorSpeeds();
	}
}
