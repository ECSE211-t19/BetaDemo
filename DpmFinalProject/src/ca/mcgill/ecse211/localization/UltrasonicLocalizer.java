package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.navigation.MainClass;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/***
 * This class implements the falling edge US localization on the EV3 platform.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class UltrasonicLocalizer implements Runnable {

	private static final int FILTER_OUT = 30;
	public static final int WALL_DISTANCE = 40;
	public static int ROTATION_SPEED = 110;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private SampleProvider usSensor;
	private Odometer odometer;
	private float[] usData;

	private int distance = 0;
	private int filter_control = 0;

	/***
	 * Constructor
	 * 
	 * 
	 * @param leftMotor,
	 *            rightMotor, TRACK, WHEEL_RAD.
	 *            
	 * @throws OdometerExceptions
	 */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) throws OdometerExceptions {
		odometer = Odometer.getOdometer();
		SensorModes us_sensor = MainClass.usSensor;
		this.usSensor = us_sensor.getMode("Distance");
		this.usData = new float[usSensor.sampleSize()];
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	/***
	 * This method performs the falling edge localization
	 * 
	 */
	public void run() {

		doFallingEdge();
	}

	/***
	 * This method starts the falling edge localization
	 * 
	 */
	public void doFallingEdge() {
		// facing the wall at the start
		double angle1;
		double angle2;

		usSensor.fetchSample(usData, 0);
		this.distance = (int) (usData[0] * 100.0);

		while (this.distance == 0.0) {
			usSensor.fetchSample(usData, 0);
			this.distance = (int) (usData[0] * 100.0);
		}

		if (this.distance < 65) {
			while (this.distance < 65) {
				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.backward();
				rightMotor.forward();

				fetchUSData();
			}
			leftMotor.stop(true);
			rightMotor.stop(false);

			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.backward();
				rightMotor.forward();

				fetchUSData();
			}
			Sound.beep();
			leftMotor.stop(true);
			rightMotor.stop(false);

			angle1 = odometer.getXYT()[2];

			while (this.distance < 65) {
				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.forward();
				rightMotor.backward();

				fetchUSData();
			}
			Sound.beep();
			leftMotor.stop(true);
			rightMotor.stop(false);

			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.forward();
				rightMotor.backward();

				fetchUSData();
			}
			Sound.beep();
			leftMotor.stop(true);
			rightMotor.stop(false);

			angle2 = odometer.getXYT()[2];

			leftMotor.rotate(-convertAngle(MainClass.WHEEL_RAD, MainClass.TRACK, 45 + ((angle2 - angle1) / 2.0)), true);
			rightMotor.rotate(convertAngle(MainClass.WHEEL_RAD, MainClass.TRACK, 45 + ((angle2 - angle1) / 2.0)),
					false);

		} else {// facing away from the walls
			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.backward();
				rightMotor.forward();

				fetchUSData();
			}
			Sound.beep();
			leftMotor.stop(true);
			rightMotor.stop(false);
			angle1 = 360 - odometer.getXYT()[2];

			while (this.distance < 65) {
				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.forward();
				rightMotor.backward();

				fetchUSData();
			}
			leftMotor.stop(true);
			rightMotor.stop(false);

			while (this.distance > WALL_DISTANCE) {
				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.forward();
				rightMotor.backward();

				fetchUSData();
			}
			Sound.beep();
			leftMotor.stop(true);
			rightMotor.stop(false);

			angle2 = odometer.getXYT()[2];

			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.rotate(-convertAngle(MainClass.WHEEL_RAD, MainClass.TRACK, 45 + ((angle1 + angle2) / 2.0)), true);
			rightMotor.rotate(convertAngle(MainClass.WHEEL_RAD, MainClass.TRACK, 45 + ((angle1 + angle2) / 2.0)),
					false);

		}

		Sound.beep();

	}

	/***
	 * This method fetches data from the ultrasonic sensor and filters it
	 * 
	 */
	private void fetchUSData() {
		usSensor.fetchSample(usData, 0); // acquire data
		int new_distance = (int) Math.abs(usData[0] * 100.0); // extract from buffer, cast to int
		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		if (new_distance >= 255 && filter_control < FILTER_OUT) {
			filter_control++;
		} else if (new_distance >= 255) {
			this.distance = new_distance;
		} else {
			filter_control = 0;
			this.distance = new_distance;
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
}
