package ca.mcgill.ecse211.ringCapture;

import java.util.concurrent.TimeUnit;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


/**
 * 
 * @author Noam Suissa
 *
 */
public class ArmController {
	private static final double BOTTOMTOFIRST = 10.0; //put correct value. distance the arm needs to cover from the bottom to the desired first level height 
	private static final double FIRSTTOSECOND = 20.0; //put correct value. distance the arm needs to cover from the bottom to the desired second level from the first
	private static final double SPOOLRADIUS = 1.0; //change
	private static final int FORWARD_SPEED = 100;
	private static double WHEELRADIUS;
	private static double TRACK;
	private double theta;
	private NXTRegulatedMotor armMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private RingColours ringColours;

	/**
	 * Initializes motors and hardware constants
	 * 
	 * @param armMotor
	 * @param leftMotor
	 * @param rightMotor
	 * @param wheelRadius
	 * @param track
	 */
	public ArmController(NXTRegulatedMotor armMotor, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double wheelRadius, double track) {
		this.armMotor = armMotor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		WHEELRADIUS = wheelRadius;
		TRACK = track;
		this.armMotor.stop();
	}

	/**
	 * perform moveArm() and moveRobot() routines for first level. If no ring was detected, move back and repeat same process for second level.
	 */
	public void run() {

		//create new ring detection object
		ringColours = new RingColours(); //change place in code depending on when you want to activate light sensor.

		moveArm(BOTTOMTOFIRST);

		boolean firstLevelDetected = moveRobot();

		//need to move arm to second level if first level has no ring
		if(!firstLevelDetected) {

			moveArm(FIRSTTOSECOND);

			moveRobot(); //same process, ignore returned boolean

		}

	}

	private void moveArm(double level) {
		armMotor.setSpeed(FORWARD_SPEED);
		armMotor.rotate(convertDistance(SPOOLRADIUS, level), false);
		armMotor.stop();
	}

	/**
	 * This method moves the robot carefully in a certain manner to retrieve and detect a ring coulour
	 * @return true or false depending on if a first level ring was detected 
	 */
	private boolean moveRobot() {

		boolean firstLevelDetected = false;
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		//advance robot to insert arm in ring hole
		leftMotor.rotate(convertDistance(WHEELRADIUS, 10), true); 
		rightMotor.rotate(convertDistance(WHEELRADIUS, 10), false);

		//dont know if this will work. want to let color sensor activate before continuing robot movement 
		try {
			TimeUnit.SECONDS.sleep(3);
		}catch(InterruptedException e) {

		}

		leftMotor.setSpeed(FORWARD_SPEED/4);
		rightMotor.setSpeed(FORWARD_SPEED/4);
		//move robot back to retrieve arm
		leftMotor.rotate(-convertDistance(WHEELRADIUS, 10), true); 
		rightMotor.rotate(-convertDistance(WHEELRADIUS, 10), false);

		//detect color and beep accordingly
		if(ringColours.colourDetected("Red")) {
			beep(4);
			firstLevelDetected = true;
		}else if(ringColours.colourDetected("Blue")) {
			beep(1);
			firstLevelDetected = true;
		}else if(ringColours.colourDetected("Green")) {
			beep(2);
			firstLevelDetected = true;
		}else if(ringColours.colourDetected("Yellow")) {
			beep(3);
			firstLevelDetected = true;
		}

		return firstLevelDetected;
	}

	/**
	 * Method to beep <code>times</code> times
	 * @param times
	 */
	private void beep(int times) {
		for(int i=0; i<times; i++) {
			Sound.beep();
		}
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Calls convertDistance() when wanting to rotate base of robot
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
