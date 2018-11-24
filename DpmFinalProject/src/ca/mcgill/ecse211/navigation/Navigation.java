package ca.mcgill.ecse211.navigation;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

import java.util.HashMap;
import java.util.Map;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/***
 * This class implements the navigation and obstacle avoidance in Lab5 on the
 * EV3 platform.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class Navigation  {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private float[] usData;
	private SampleProvider usDistance;
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_WIDTH = 30.48;
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	int iterator = 0;
	private Odometer odometer;
	private OdometerData odoData;

	//
	/***
	 * Constructor
	 * 
	 * @param wifi
	 * 
	 */
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD, Wifi wifi) throws OdometerExceptions { // constructor
		this.odometer = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		odoData = OdometerData.getOdometerData();
		odoData.setXYT(0, 0, 0);

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

		SensorModes usSensor = MainClass.usSensor; // usSensor is the instance
		this.usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
		this.usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned

	}


	/***
	 * This makes the robot move forward
	 * 
	 * 
	 * @param distance
	 */
	void travelForward(double distance) {
		// drive forward required distance
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

	}

	/***
	 * This method moves the robot to the set waypoint
	 * 
	 * 
	 * @param x,
	 *            y
	 */
	public void travelTo(double x, double y, int forwardSpeed, int turnSpeed) {
		currentX = odometer.getXYT()[0];// get the position on the board
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];

		dx = x - currentX;
		dy = y - currentY;
		distanceToTravel = Math.hypot(dx, dy);
		if (dy >= 0) {
			dt = Math.atan(dx / dy);
		} else if (dy <= 0 && dx >= 0) {
			dt = Math.atan(dx / dy) + Math.PI;
		} else {
			dt = Math.atan(dx / dy) - Math.PI;
		}

		double differenceInTheta = (dt * 180 / Math.PI - currentT); // robot has to turn "differenceInTheta",

		// turn the robot to the desired direction
		turnTo(differenceInTheta, turnSpeed);

		// drive forward required distance
		leftMotor.setSpeed(forwardSpeed);
		rightMotor.setSpeed(forwardSpeed);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);

		rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), false);

	}


	/***
	 * This method makes the robot turn to the minimum specified angle
	 * 
	 * 
	 * @param theta
	 */
	public void turnTo(double theta, int speed) {
		if (theta > 180) { // angle convention. the robot should turn in direction
			theta = 360 - theta;
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
		} else if (theta < -180) {
			theta = 360 + theta;
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		} else {
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
	}

	/***
	 * This method checks if the robot is navigating
	 * 
	 * @return boolean returns if the robot is still navigating
	 */
	boolean isNavigating() {
		if ((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else
			return false;

	}


	/***
	 * This method converts the specified distance to an angle
	 * 
	 * 
	 * @param radius,
	 *            distance
	 * @return int returns the angle
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/***
	 * This method converts the specified angle to a distance
	 * 
	 * 
	 * @param radius,
	 *            width, angle
	 * @return int returns the distance
	 */
	static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
