package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.localization.FinalLightLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/***
 * This class implements navigation to the required waypoints in the map.
 * 
 * @authorAbedAtassi
 */
public class Pilot implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	public static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_WIDTH = 30.48;
	private OdometerData odoData;
	Wifi wifi;
	int StartingCorner;
	int[] StartXY;
	int tunnel[][];
	int SearchZone[][];
	int tree[];
	int team;
	Navigation navigation;
	FinalLightLocalizer finalLightLocalizer;

	/***
	 * Constructor
	 * 
	 * 
	 * @param leftMotor,
	 *            rightMotor, TRACK, WHEEL_RAD, finalLightLocalizer, navigation and
	 *            wifi
	 * @throws OdometerExceptions
	 */
	public Pilot(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD, Wifi wifi, Navigation navigation, FinalLightLocalizer finalLightLocalizer)
			throws OdometerExceptions {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.wifi = wifi;
		this.team = wifi.getTeam();
		this.StartingCorner = wifi.getStartingCorner(team);
		this.StartXY = wifi.getStartingCornerXY();
		this.tunnel = wifi.getTunnel();
		this.tree = wifi.getRingSet();
		odoData = OdometerData.getOdometerData();
		this.navigation = navigation;
		this.finalLightLocalizer = finalLightLocalizer;

	}

	/***
	 * This method runs all the methods required to navigate to the different
	 * waypoints
	 * 
	 */
	public void run() {
		try {
			this.odoData = Odometer.getOdometer();
		} catch (OdometerExceptions e1) {

			e1.printStackTrace();
		}
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.setAcceleration(200); // reduce the acceleration

		}

		switch (StartingCorner) { // Set start coordinates depending on starting corner

		case 0:
			odoData.setXYT(StartXY[0] * TILE_WIDTH, StartXY[1] * TILE_WIDTH, 0);
			break;

		case 1:
			odoData.setXYT(StartXY[0] * TILE_WIDTH, StartXY[1] * TILE_WIDTH, 270);
			break;

		case 2:
			odoData.setXYT(StartXY[0] * TILE_WIDTH, StartXY[1] * TILE_WIDTH, 180);
			break;

		case 3:
			odoData.setXYT(StartXY[0] * TILE_WIDTH, StartXY[1] * TILE_WIDTH, 90);
			break;
		}

		travelToTunnel();
		Sound.beep();
		Sound.beep();
		Sound.beep();

		travelThroughTunnel();

		travelToRingSet();
		Sound.beep();
		Sound.beep();
		Sound.beep();

		localizeBeforeRingSet();

	}

	/**
	 * This method drives the robot to the tunnel
	 */
	public void travelToTunnel() {

		if (wifi.isTunnelVertical()) { // Check orientation of the tunnel

			switch (StartingCorner) { // check the starting corner

			case 0:
				navigation.travelTo((tunnel[0][0]) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
				finalLightLocalizer.doNavLocalization((tunnel[0][0]) * TILE_WIDTH, StartXY[1] * TILE_WIDTH);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[1][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:
				navigation.travelTo((tunnel[1][0]) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
				finalLightLocalizer.doNavLocalization((tunnel[1][0]) * TILE_WIDTH, StartXY[1] * TILE_WIDTH);
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, (tunnel[1][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			case 2:
				navigation.travelTo((tunnel[1][0]) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
				finalLightLocalizer.doNavLocalization((tunnel[1][0]) * TILE_WIDTH, StartXY[1] * TILE_WIDTH);
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			case 3:
				navigation.travelTo((tunnel[0][0]) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
				finalLightLocalizer.doNavLocalization((tunnel[0][0]) * TILE_WIDTH, StartXY[1] * TILE_WIDTH);

				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, StartXY[1] * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			}
		} else {

			switch (StartingCorner) {

			case 0:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[2][1]) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				finalLightLocalizer.doNavLocalization((StartXY[0]) * TILE_WIDTH, (tunnel[2][1]) * TILE_WIDTH);
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[0][1]) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				finalLightLocalizer.doNavLocalization((StartXY[0]) * TILE_WIDTH, (tunnel[0][1]) * TILE_WIDTH);
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[0][1] + 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[2][0] + 1) * TILE_WIDTH, (tunnel[0][1] + 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 2:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1]) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				finalLightLocalizer.doNavLocalization((StartXY[0]) * TILE_WIDTH, (tunnel[3][1]) * TILE_WIDTH);
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[2][0] + 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 3:
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1]) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				finalLightLocalizer.doNavLocalization((StartXY[0]) * TILE_WIDTH, (tunnel[3][1]) * TILE_WIDTH);
				navigation.travelTo((StartXY[0]) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			}
		}
	}

	/**
	 * This method drives the robot through the tunnel
	 */
	public void travelThroughTunnel() {

		if (wifi.isTunnelVertical()) {
			switch (StartingCorner) {

			case 0:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;
			case 1:
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, (tunnel[3][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 2:
				navigation.travelTo((tunnel[1][0] - 0.5) * TILE_WIDTH, (tunnel[1][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 3:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[0][0] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			}

		}

		else {

			switch (StartingCorner) {

			case 0:
				navigation.travelTo((tunnel[2][0] + 1) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 1:
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[0][1] + 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 2:
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 3:
				navigation.travelTo((tunnel[2][0] + 1) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;
			}
		}
	}

	/**
	 * This method moves the robot to the ringset
	 * 
	 */
	public void travelToRingSet() {

		if (wifi.isTunnelVertical()) {

			int uLX = tunnel[0][0];

			if ((uLX) > tree[0]) {
				navigation.travelTo((tree[0] + 1) * TILE_WIDTH, (tree[1]) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);

			}

			else {
				navigation.travelTo((tree[0] - 1) * TILE_WIDTH, (tree[1]) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			}

		}

		else {

			int uLY = tunnel[0][1];

			if (uLY > tree[1]) {
				navigation.travelTo((tree[0]) * TILE_WIDTH, (tree[1] + 1) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			}

			else {
				navigation.travelTo((tree[0]) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tree[0]) * TILE_WIDTH, (tree[1] - 1) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			}

		}

	}

	/***
	 * This method localizes the robot before trying to grab rings
	 * 
	 */
	public void localizeBeforeRingSet() {

		if (wifi.isTunnelVertical()) {

			int uLX = tunnel[0][0];
			int uRX = tunnel[2][0];

			if ((uLX + uRX) / 2 > tree[0]) {
				finalLightLocalizer.doNavLocalization((tree[0] + 1) * TILE_WIDTH, (tree[1]) * TILE_WIDTH);
			}

			else {
				finalLightLocalizer.doNavLocalization((tree[0] - 1) * TILE_WIDTH, (tree[1]) * TILE_WIDTH);
			}

		}

		else {
			int uLY = tunnel[0][1];

			if (uLY > tree[1]) {
				finalLightLocalizer.doNavLocalization((tree[0]) * TILE_WIDTH, (tree[1] + 1) * TILE_WIDTH);
			}

			else {
				finalLightLocalizer.doNavLocalization((tree[0]) * TILE_WIDTH, (tree[1] - 1) * TILE_WIDTH);
			}

		}
	}

	/***
	 * This method makes the robot move back to the tunnel
	 * 
	 */
	public void travelBackToTunnel() {

		if (wifi.isTunnelVertical()) {

			switch (StartingCorner) {

			case 0:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tree[1]) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[2][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tree[1]) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[2][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 2:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tree[1]) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[0][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 3:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tree[1]) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[0][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			}
		} else {
			switch (StartingCorner) {

			case 0:
				navigation.travelTo((tree[0]) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:
				navigation.travelTo((tree[0]) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 2:
				navigation.travelTo((tree[0]) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 3:
				navigation.travelTo((tree[0]) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[3][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			}

		}

	}

	/***
	 * This method makes the robot move back through the tunnel
	 * 
	 */
	public void travelBackThroughTunnel() {

		if (wifi.isTunnelVertical()) {
			switch (StartingCorner) {

			case 0:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[0][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 1) * TILE_WIDTH, (tunnel[0][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[0][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 1) * TILE_WIDTH, (tunnel[0][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 2:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[2][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 1) * TILE_WIDTH, (tunnel[2][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 3:
				navigation.travelTo((tunnel[0][0] + 0.5) * TILE_WIDTH, (tunnel[2][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 1) * TILE_WIDTH, (tunnel[2][1] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			}

		}

		else {
			switch (StartingCorner) {

			case 0:
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[0][0] + 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[0][0] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 1:
				navigation.travelTo((tunnel[0][1] + 1) * TILE_WIDTH, (tunnel[0][0] + 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][1] + 1) * TILE_WIDTH, (tunnel[0][0] + 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);

				return;

			case 2:
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] - 1) * TILE_WIDTH, (tunnel[2][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;

			case 3:
				navigation.travelTo((tunnel[0][0] + 1) * TILE_WIDTH, (tunnel[2][1] - 0.5) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				navigation.travelTo((tunnel[0][0] + 1) * TILE_WIDTH, (tunnel[2][1] - 1) * TILE_WIDTH, FORWARD_SPEED,
						ROTATE_SPEED);
				return;
			}
		}

	}

	/***
	 * This method makes the robot move back to the starting corner
	 * 
	 */
	public void travelBackToStartingCorner() {

		navigation.travelTo((StartXY[0]) * TILE_WIDTH, (StartXY[1]) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
		navigation.travelForward(5);

	}

}
