package resource;

public class ResourceFunctions {

	public static double tickToAngle(int tick) {
		return (double) tick * 360.0 / 4096.0;
	}

	public static int angleToTick(double angle) {
		return (int) Math.round(angle * 4096.0 / 360.0);
	}

	public static double speedToPercent(double speed) {
		return speed / 1023;
	}

	/**
	 * Gets angle between 0 and 360
	 * 
	 * @param angle
	 *            angle
	 * @return 0 <= angle < 360
	 */
	public static double putAngleInRange(double angle) {
		return (angle + 36000) % 360;
	}

	/**
	 * 
	 * Returns the difference between two angles between -180 and 180. If using for
	 * PID for error, target angle minus current angle.
	 * 
	 * @param angle1
	 *            first angle
	 * @param angle2
	 *            second angle
	 * @return difference
	 */
	public static double continuousAngleDif(double angle1, double angle2) { // target - current
		double dif = putAngleInRange(angle1) - putAngleInRange(angle2);
		dif = putAngleInRange(dif);
		if (dif > 180)
			dif = dif - 360;
		return dif;
	}

	/**
	 * Equality of two doubles within 0.001
	 * 
	 * @param a
	 *            first double
	 * @param b
	 *            second double
	 * @return almost equal to
	 */
	public static boolean equals(double a, double b) {
		return (Math.abs(a - b) < 0.001);
	}

	/**
	 * Ensures a number is between a min and max value
	 * 
	 * @param val
	 *            number
	 * @param min
	 *            lowest number should be
	 * @param max
	 *            highest number should be
	 * @return number between the min and max
	 */
	public static double putNumInAbsoluteRange(double val, double min, double max) {
		return Math.max(Math.min(val, max), min);
		// if (val > max) {
		// 	return max;
		// }
		// else if (val < min) {
		// 	return min;
		// }
		// else {
		// 	return val;
		// }

	}

	/**
	 * returns derivative of the cosine function of the form a*cos(b*t) at a time t
	 * b*t is in radians
	 * 
	 * @param a
	 *            leading coefficient
	 * @param b
	 *            coefficient of t
	 * @param c
	 *            initial condition
	 * @param t
	 *            time
	 * @return derivative of the function at the point t
	 */
	public static double cosineDerivative(double a, double b, double c, double t) {
		return -a * b * Math.sin(b * t + c);
	}

	/**
	 * returns derivative of the sine function of the form a*sin(b*t) at a time t
	 * b*t is in radians
	 * 
	 * @param a
	 *            leading coefficient
	 * @param b
	 *            coefficient of t
	 * @param c
	 *            initial condition
	 * @param t
	 *            time
	 * @return derivative of the function at the point t
	 */
	public static double sineDerivative(double a, double b, double c, double t) {
		return a * b * Math.cos(b * t + c);
	}
}