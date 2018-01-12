
package org.usfirst.frc.team4276.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import autonomous.robotPositionFinder;

public class Robot extends SampleRobot {

	
	robotPositionFinder rpf;
	public Robot() {
		rpf = new robotPositionFinder(0,1,2,3);//placeholders TEST
		rpf.start();
	}

	public void robotInit() {

	}

	public void autonomous() {

	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {

			Timer.delay(0.005);
		}
	}

	public void test() {
	}
}
