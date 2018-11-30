package ca.mcgill.ecse211.wifi;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * This class passes all the required parameters of the map to the robot
 *
 * @author @AbedAtassi
 *
 */
public class Wifi {

	private static final String SERVER_IP = "192.168.2.2";
	private static final int TEAM_NUMBER = 19;
	private Map data;
	private int team;

	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

	public Wifi() {
		this.getMapData();
	}

	@SuppressWarnings("rawtypes")
	public void getMapData() {

		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data
		try {

			data = conn.getData();

		} catch (Exception e) {

			System.err.println("Error: " + e.getMessage());
		}

	}

	/**
	 * This method gets the color of the team.
	 * 
	 * @return int
	 */
	public int getTeam() {

		team = ((Long) data.get("RedTeam")).intValue();

		if (team == 19) { // if team color is red return 1 else return 0

			return 1;

		} else {

			return 0;
		}
	}

	/**
	 * This methods gets the starting corner number of the team.
	 * 
	 * @return int
	 */
	public int getStartingCorner(int team) {

		team = getTeam();

		if (team == 1) {

			return ((Long) data.get("RedCorner")).intValue();

		} else {

			return ((Long) data.get("GreenCorner")).intValue();
		}
	}

	/**
	 * This method gets the starting corner coordinates
	 *
	 * @return startXY
	 */
	public int[] getStartingCornerXY() {

		int[] startXY = { 0, 0 };

		team = getTeam();

		switch (getStartingCorner(team)) {

		case 0:
			startXY[0] = 1;
			startXY[1] = 1;
			break;

		case 1:
			startXY[0] = 14;
			startXY[1] = 1;
			break;

		case 2:
			startXY[0] = 14;
			startXY[1] = 8;
			break;

		case 3:
			startXY[0] = 1;
			startXY[1] = 8;
			break;
		}
		return startXY;
	}

	/**
	 * This method gets the coordinates of the starting area.
	 * 
	 * @return startArea
	 */
	public int[][] getStartArea() {

		team = getTeam();

		if (team == 1) {

			int llx = ((Long) data.get("Green_LL_x")).intValue();
			int lly = ((Long) data.get("Green_LL_y")).intValue();
			int urx = ((Long) data.get("Green_UR_x")).intValue();
			int ury = ((Long) data.get("Green_UR_y")).intValue();

			int[][] startArea = { { llx, lly }, { urx, lly }, { urx, ury }, { llx, ury } };

			return startArea;

		} else {

			int lLX = ((Long) data.get("Red_LL_x")).intValue();
			int lLY = ((Long) data.get("Red_LL_y")).intValue();
			int uRX = ((Long) data.get("Red_UR_x")).intValue();
			int uRY = ((Long) data.get("Red_UR_y")).intValue();

			int[][] startArea = { { lLX, lLY }, { uRX, lLY }, { uRX, uRY }, { lLX, uRY } };

			return startArea;
		}

	}

	/**
	 * This method gets the coordinates of the tunnel
	 * 
	 * @return tunnel
	 */
	public int[][] getTunnel() {

		int llx, lly, urx, ury;
		team = getTeam();

		if (team == 0) {

			llx = ((Long) data.get("TNR_LL_x")).intValue();
			lly = ((Long) data.get("TNR_LL_y")).intValue();
			urx = ((Long) data.get("TNR_UR_x")).intValue();
			ury = ((Long) data.get("TNR_UR_y")).intValue();

			int[][] tunnel = { { llx, lly }, { urx, lly }, { urx, ury }, { llx, ury } };

			return tunnel;

		} else {

			llx = ((Long) data.get("TNG_LL_x")).intValue();
			lly = ((Long) data.get("TNG_LL_y")).intValue();
			urx = ((Long) data.get("TNG_UR_x")).intValue();
			ury = ((Long) data.get("TNG_UR_y")).intValue();
			int[][] tunnel = { { llx, lly }, { urx, lly }, { urx, ury }, { llx, ury } };

			return tunnel;

		}
	}

	/**
	 * This method returns the island coordinates
	 * 
	 * @return island
	 */
	public int[][] getIsland() {

		int llx, lly, urx, ury;

		llx = ((Long) data.get("Island_LL_x")).intValue();
		lly = ((Long) data.get("Island_LL_y")).intValue();
		urx = ((Long) data.get("Island_UR_x")).intValue();
		ury = ((Long) data.get("Island_UR_y")).intValue();

		int[][] island = { { llx, lly }, { urx, lly }, { urx, ury }, { llx, ury } };

		return island;
	}

	/**
	 * This method gets the ring set coordinates
	 * 
	 * @return ringSet
	 */
	public int[] getRingSet() {

		int tr_x, tr_y, tg_x, tg_y;

		if (team == 0) {

			tr_x = ((Long) data.get("TR_x")).intValue();
			tr_y = ((Long) data.get("TR_y")).intValue();

			int[] ringSet = { tr_x, tr_y };

			return ringSet;

		} else {

			tg_x = ((Long) data.get("TG_x")).intValue();
			tg_y = ((Long) data.get("TG_y")).intValue();

			int[] ringSet = { tg_x, tg_y };

			return ringSet;

		}
	}

	/**
	 * This method returns the orientation of the tunnel
	 *
	 * @return boolean
	 */
	public boolean isTunnelVertical() {

		team = getTeam();

		int llx, urx;

		int ilx = ((Long) data.get("Island_LL_x")).intValue();
		int iux = ((Long) data.get("Island_UR_x")).intValue();

		if (team == 1) {

			llx = ((Long) data.get("Red_LL_x")).intValue();
			urx = ((Long) data.get("Red_UR_x")).intValue();

			if (getStartingCorner(team) == 0 || getStartingCorner(team) == 3) {

				if (urx > ilx) {
					return true;
				} else {
					return false;
				}

			} else {
				if (llx > iux) {
					return false;
				} else {
					return true;
				}
			}
		} else {
			llx = ((Long) data.get("Green_LL_x")).intValue();
			urx = ((Long) data.get("Green_UR_x")).intValue();

			if (getStartingCorner(team) == 0 || getStartingCorner(team) == 3)
				if (urx > ilx) {
					return true;
				} else {
					return false;
				}

			else {
				if (llx > iux) {
					return false;
				} else {
					return true;
				}
			}

		}

	}

}
