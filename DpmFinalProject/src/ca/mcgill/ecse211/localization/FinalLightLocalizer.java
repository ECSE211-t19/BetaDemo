package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.ev3.LocalEV3;

/***
 * This class implements the light localization in Lab4 on the EV3 platform.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class FinalLightLocalizer implements Runnable {

	private SampleProvider color_sample_provider;
	private float[] color_samples;
	private float light_value;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final double d = 13.1; // distance between the center of the robot and the light sensor
	private static final double TILE_WIDTH = 30.48;
	private static final int ROTATE_SPEED = 110;
	private double TRACK;
	private double WHEEL_RAD;
	private Odometer odoData;
	private float prev_red;
	private float curr_red;
	private int startCorner;
	private double dX;
	private double dY;
	private float first, second, third, fourth, angleCorrection;

	/***
	 * Constructor
	 * 
	 * 
	 * @param leftMotor, rightMotor, TRACK, WHEEL_RAD
	 */
	public FinalLightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		EV3ColorSensor colour_sensor = MainClass.lineSensor;
		this.color_sample_provider = colour_sensor.getMode("ColorID");
		this.color_samples = new float[colour_sensor.sampleSize()];
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.startCorner = startCorner;
	}

	public void run() {
		try {
			this.odoData = Odometer.getOdometer();
		} catch (OdometerExceptions e1) {

			// e1.printStackTrace();
		}
		// wait
//		try {
//			Thread.sleep(2000);
//		} catch (InterruptedException e) {
//
//		}

		do_localization();
	}

	/***
	 * This method starts the light localization on the robot
	 *
	 * 
	 * 
	 */
	public void do_localization() {
		odoData.setX(0);
		odoData.setY(0);
		odoData.setTheta(0);
		int numberLines = 0;
		double[] angles = new double[4];
		boolean line = false;

		//angleCorrection = odoData.prev_gyro_value;
		
		Sound.beep();
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
		rightMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
		
		
		prev_red = fetchUSData();
		
		while (numberLines < 4) {
			curr_red = fetchUSData();
			
			if ( prev_red - curr_red > 3.5) { //3.5
				
//				angles[numberLines] = Math.abs(odoData.prev_gyro_value - angleCorrection);
				angles[numberLines] = odoData.getXYT()[2];
				//System.out.println(prev_red);
				//System.out.println(curr_red);
				Sound.beep();
				numberLines++;
			}
			
			prev_red = curr_red;
		}
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
//		try {
//			Thread.sleep(2000);
//		} catch (InterruptedException e) {
//
//		}
		// do calculations
		double deltaX = angles[2] - angles[0];
		double deltaY = angles[3] - angles[1];

		double xZero = d * Math.cos(Math.toRadians(deltaY / 2));
		double yZero = d * Math.cos(Math.toRadians(deltaX / 2));

		
		MainClass.navigation.travelTo(xZero, yZero, ROTATE_SPEED, ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), true);
        rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), false);
		//MainClass.navigation.turnTo(270, ROTATE_SPEED);
		//odoData.setXYT(7 * TILE_WIDTH, TILE_WIDTH, 270);

		//odoData.setXYT(7, 1, theta);
		/*
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		
//		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, ((angles[0] + angles[2]) / 2) - odoData.getXYT()[2] - 60), true);
//		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, ((angles[0] + angles[2]) / 2) - odoData.getXYT()[2] - 60), false);
		
		
		leftMotor.backward();
		rightMotor.forward();
		
		while (true) {
			curr_red = fetchUSData();
			
			if ( prev_red - curr_red > 3.5) {
				Sound.beep();
				leftMotor.stop(true);
				rightMotor.stop(false);
				break;
			}
			prev_red = curr_red;
		}
		
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 3), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 3), false);
		*/
		
		
		/*
		double[] position = { TILE_WIDTH, TILE_WIDTH, 0 };
		boolean[] set = { true, true, true };
		odoData.setPosition(position);
		*/
	}

	/***
	 * This method fetches the US data
	 * 
	 */
	public float fetchUSData() {
		color_sample_provider.fetchSample(color_samples, 0);

		return (color_samples[0] * 100);

	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}