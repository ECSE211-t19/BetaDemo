package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;

/***
 * This class implements the light localization.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class LightLocalizer implements Runnable {

	private SampleProvider color_sample_provider;
	private float[] color_samples;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final double d = 13.3; // distance between the center of the robot and the light sensor
	private static final int ROTATE_SPEED = 110;
	private double TRACK;
	private double WHEEL_RAD;
	private Odometer odoData;
	private float prev_red;
	private float curr_red;
	private double deltaX, deltaY, xZero, yZero;
	private EV3GyroSensor gyroSensor;
	private Navigation navigation;
	private float errorMargin = 150;
	private float black = 220;

	/***
	 * Constructor
	 * 
	 * 
	 * @param leftMotor,
	 *            rightMotor, TRACK, WHEEL_RAD, gyroSensor, navigation and wifi
	 * @throws OdometerExceptions
	 */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD, EV3GyroSensor gyroSensor, Navigation navigation) throws OdometerExceptions {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odoData = Odometer.getOdometer();
		EV3ColorSensor colour_sensor = MainClass.lineSensor;
		this.color_sample_provider = colour_sensor.getMode("Red");
		this.color_samples = new float[colour_sensor.sampleSize()];
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.gyroSensor = gyroSensor;
		this.navigation = navigation;
	}

	/***
	 * This method runs all the methods required for light localization on the robot
	 *
	 */
	public void run() {

		do_localization();
	}

	/***
	 * This method starts the light localization on the robot
	 *
	 */
	public void do_localization() {
		odoData.setX(0);
		odoData.setY(0);
		odoData.setTheta(0);
		int numberLines = 0;
		double[] angles = new double[4];

		Sound.beep();

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
		rightMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
		prev_red = fetchColorData();

		while (numberLines < 4) {
			curr_red = fetchColorData();

			if (prev_red - curr_red > 19 && isDetectingLine(10)) {
				angles[numberLines] = odoData.getXYT()[2];
				Sound.beep();
				numberLines++;
			}

			prev_red = curr_red;
		}

		leftMotor.stop(true);
		rightMotor.stop(false);

		// do calculations
		double deltaX = angles[2] - angles[0];
		double deltaY = angles[3] - angles[1];

		double xZero = d * Math.cos(Math.toRadians(deltaY / 2));
		double yZero = d * Math.cos(Math.toRadians(deltaX / 2));

		navigation.travelTo(xZero, yZero, ROTATE_SPEED, ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), false);
		resetGyro(gyroSensor);
	}

	/***
	 * This method starts the light localization on the robot
	 *
	 */
	public void doNavLocalization(double toX, double toY) {

		int case_num = 0;

		double[] currXYT = odoData.getXYT();

		// straighten the robot out
		if (currXYT[2] > 315.0 || currXYT[2] < 45.0) {
			if (currXYT[2] >= 0) {
				navigation.turnTo(-currXYT[2], ROTATE_SPEED);
			} else {
				navigation.turnTo(360.0 - currXYT[2], ROTATE_SPEED);
			}
		} else if (currXYT[2] > 45.0 && currXYT[2] < 135.0) {
			navigation.turnTo(90.0 - currXYT[2], ROTATE_SPEED);
		} else if (currXYT[2] > 135.0 && currXYT[2] < 225.0) {
			navigation.turnTo(180.0 - currXYT[2], ROTATE_SPEED);
		} else if (currXYT[2] > 225.0 && currXYT[2] < 315.0) {
			navigation.turnTo(270.0 - currXYT[2], ROTATE_SPEED);
		}

		odoData.setTheta(0);

		// Determine four cases of localization, four different quadrants
		if (currXYT[0] > toX && currXYT[1] > toY) {
			case_num = 1;
		} else if (currXYT[0] <= toX && currXYT[1] > toY) {
			case_num = 2;
		} else if (currXYT[0] <= toX && currXYT[1] <= toY) {
			case_num = 3;
		} else if (currXYT[0] > toX && currXYT[1] <= toY) {
			case_num = 4;
		}

		int numberLines = 0;
		double[] angles = new double[4];

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// different turning orientation per case
		switch (case_num) {
		case 1:
		case 4:
			leftMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
			break;
		case 2:
		case 3:
			leftMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
			rightMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
			break;

		default:
			Sound.beepSequenceUp();
			break;
		}

		prev_red = fetchColorData();
		while (numberLines < 4) {
			curr_red = fetchColorData();

			if (prev_red - curr_red > 15 && isDetectingLine(10)) {
				angles[numberLines] = odoData.getXYT()[2];
				Sound.beep();
				numberLines++;
			}

			prev_red = curr_red;
		}
		leftMotor.stop(true);
		rightMotor.stop(false);

		switch (case_num) {
		case 1:
			// do calculations
			deltaX = 360 - (angles[0] - angles[2]);
			deltaY = angles[1] - angles[3];
			xZero = d * Math.cos(Math.toRadians(deltaY / 2));
			yZero = d * Math.cos(Math.toRadians(deltaX / 2));

			navigation.travelTo(currXYT[0] - xZero, currXYT[1] - yZero, ROTATE_SPEED, ROTATE_SPEED);

			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(yZero / xZero)) + 90), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(yZero / xZero)) + 90), false);
			break;

		case 2:
			// do calculations
			deltaX = 360 - (angles[2] - angles[0]);
			deltaY = angles[3] - angles[1];
			xZero = d * Math.cos(Math.toRadians(deltaY / 2));
			yZero = d * Math.cos(Math.toRadians(deltaX / 2));

			navigation.travelTo(currXYT[0] + xZero, currXYT[1] - yZero, ROTATE_SPEED, ROTATE_SPEED);

			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(yZero / xZero)) + 90), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(yZero / xZero)) + 90), false);
			break;

		case 3:
			// do calculations
			deltaX = angles[2] - angles[0];
			deltaY = angles[3] - angles[1];
			xZero = d * Math.cos(Math.toRadians(deltaY / 2));
			yZero = d * Math.cos(Math.toRadians(deltaX / 2));

			navigation.travelTo(currXYT[0] + xZero, currXYT[1] + yZero, ROTATE_SPEED, ROTATE_SPEED);

			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), false);
			break;

		case 4:
			// do calculations
			deltaX = angles[0] - angles[2];
			deltaY = angles[1] - angles[3];
			xZero = d * Math.cos(Math.toRadians(deltaY / 2));
			yZero = d * Math.cos(Math.toRadians(deltaX / 2));

			navigation.travelTo(currXYT[0] - xZero, currXYT[1] + yZero, ROTATE_SPEED, ROTATE_SPEED);

			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), false);
			break;

		default:
			Sound.beepSequenceUp();
			break;

		}

		odoData.setX(toX);
		odoData.setY(toY);

		// set angle to closest angle in multiple of 90 degrees
		if (currXYT[2] > 315.0 || currXYT[2] < 45.0) {
			odoData.setTheta(0);
		} else if (currXYT[2] > 45.0 && currXYT[2] < 135.0) {
			odoData.setTheta(90);
		} else if (currXYT[2] > 135.0 && currXYT[2] < 225.0) {
			odoData.setTheta(180);
		} else if (currXYT[2] > 225.0 && currXYT[2] < 315.0) {
			odoData.setTheta(270);
		}
	}

	/***
	 * This method fetches the US data
	 * 
	 */
	public float fetchColorData() {
		color_sample_provider.fetchSample(color_samples, 0);

		return (color_samples[0] * 1000);

	}

	/***
	 * This method returns if a black line is detected or not
	 * 
	 * 
	 * @param sampleCount
	 * @return bool
	 */
	public boolean isDetectingLine(int sampleCount) {
		float sum = 0;

		for (int i = 0; i < sampleCount; i++) {
			sum += fetchColorData();
		}

		float avg = sum / sampleCount;

		if (avg > black - errorMargin && avg < black + errorMargin) {
			return true;
		} else {
			return false;
		}
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
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/***
	 * This method resets the gyroscope
	 * 
	 * 
	 * @param gyro
	 */
	public static void resetGyro(EV3GyroSensor gyro) {
		gyro.reset();

	}
}