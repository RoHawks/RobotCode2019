package resource;

import java.awt.geom.Point2D;

public class Vector {
	private double xVal;
	private double yVal;

	/**
	 * create vector with x, y component
	 * 
	 * @param xVal
	 *            x component of vector
	 * @param yVal
	 *            y component of vector
	 */
	public Vector(double xVal, double yVal) {
		this.xVal = xVal;
		this.yVal = yVal;
	}

	/**
	 * create vector equal to another vector
	 * 
	 * @param other
	 *            vector to copy
	 */
	public Vector(Vector other) {
		this(other.getX(), other.getY());
	}

	/**
	 * creates null vector
	 */
	public Vector() {
		this(0, 0);
	}

	/**
	 * creates polar vector
	 * 
	 * @param angle
	 *            angle of vector
	 * @param total
	 *            magnitude of vector
	 * @return
	 */
	public static Vector createPolar(double angle, double total) {
		Vector v = new Vector();
		v.setPolar(angle, total);
		return v;
	}

	/**
	 * adds two vectors
	 * 
	 * @param v1
	 *            vector
	 * @param v2
	 *            vector
	 * @return sum vector
	 */
	public static Vector add(Vector v1, Vector v2) {
		Vector v = new Vector();
		v.addCartesian(v1);
		v.addCartesian(v2);
		return v;
	}

	/**
	 * x component getter
	 * 
	 * @return x value of vector
	 */
	public double getX() {
		return this.xVal;
	}

	/**
	 * y component getter
	 * 
	 * @return y component of vector
	 */
	public double getY() {
		return this.yVal;
	}

	/**
	 * magnitude getter
	 * 
	 * @return magnitude of vector
	 */
	public double getMagnitude() {
		return Math.hypot(xVal, yVal);
	}

	/**
	 * angle getter
	 * 
	 * @return angle of the vector
	 */
	public double getAngle() {
		return (((Math.toDegrees(Math.atan2(yVal, xVal))) + 3600) % 360);
	}

	/**
	 * vector as the point from the origin
	 * 
	 * @return point (x component, y component)
	 */
	public Point2D.Double asPoint() {
		return new Point2D.Double(this.getX(), this.getY());
	}

	/**
	 * x component setter
	 * 
	 * @param xVal
	 *            value to set x
	 */
	private void setX(double xVal) {
		this.xVal = xVal;
	}

	/**
	 * y component setter
	 * 
	 * @param yVal
	 *            value to set y
	 */
	private void setY(double yVal) {
		this.yVal = yVal;
	}

	/**
	 * set x and y components
	 * 
	 * @param xVal
	 *            value to set x
	 * @param yVal
	 *            value to set y
	 */
	public void setCartesian(double xVal, double yVal) {
		this.setX(xVal);
		this.setY(yVal);
	}

	/**
	 * add vector to this one
	 * 
	 * @param xVal
	 *            x component of vector to add
	 * @param yVal
	 *            y component of vector to add
	 */
	public void addCartesian(double xVal, double yVal) {
		this.setCartesian(this.xVal + xVal, this.yVal + yVal);
	}

	/**
	 * add vector to this one
	 * 
	 * @param v
	 *            vector to add
	 */
	public void addCartesian(Vector v) {
		this.addCartesian(v.getX(), v.getY());
	}

	/**
	 * set polar coordinates
	 * 
	 * @param angle
	 *            angle of vector
	 * @param total
	 *            magnitude of vector
	 */
	public void setPolar(double angle, double total) {
		this.setCartesian(Math.cos(Math.toRadians(angle)) * total, Math.sin(Math.toRadians(angle)) * total);
	}

	/**
	 * set angle
	 * 
	 * @param angle
	 *            angle to set vector
	 */
	public void setAngle(double angle) {
		this.setPolar(angle, this.getMagnitude());
	}

	/**
	 * set magnitude
	 * 
	 * @param total
	 *            magnitude of vector
	 */
	public void setTotal(double total) {
		this.setPolar(this.getAngle(), total);
	}

	/**
	 * add polar vector to this one
	 * 
	 * @param angle
	 *            angle of polar vector to add
	 * @param total
	 *            magnitude of polar vector to add
	 */
	public void addPolar(double angle, double total) {
		this.setPolar(this.getAngle() + angle, this.getMagnitude() + total);
	}

	/**
	 * multiply by scalar
	 * 
	 * @param scaleAmount
	 *            scalar to multiply vector
	 */
	public void scaleTotal(double scaleAmount) {
		this.xVal *= scaleAmount;
		this.yVal *= scaleAmount;
	}

	/**
	 * dot product of two vectors
	 * 
	 * @param v
	 *            vector to dot this one
	 * @return dot product
	 */
	public double dot(Vector v) {
		return dot(this, v);
	}

	/**
	 * dot product of two vectors
	 * 
	 * @param a
	 *            first vector
	 * @param b
	 *            second vector
	 * @return dot product
	 */
	public static double dot(Vector a, Vector b) {
		return a.getX() * b.getX() + a.getY() * b.getY();
	}

	/**
	 * projection length of some Vector onto this
	 * 
	 * @param v
	 *            Vector to project onto this
	 * @return projection length of Vector v onto this
	 */
	public double projectionLengthFrom(Vector v) {
		return projectionLength(v, this);
	}

	/**
	 * projection length of this onto some other Vector
	 * 
	 * @param v
	 *            Vector to project this onto
	 * @return projection length of this onto Vector v
	 */
	public double projectionLengthOnto(Vector v) {
		return projectionLength(this, v);
	}

	/**
	 * projection length of Vector a onto Vector b
	 * 
	 * @param a
	 *            Vector to project
	 * @param b
	 *            Vector projected onto
	 * @return length of the projection
	 */
	public static double projectionLength(Vector a, Vector b) {
		return dot(a, b) / b.getMagnitude();
	}

	/**
	 * angle between two vectors
	 * 
	 * @param v
	 *            vector to find angle between
	 * @return angle
	 */
	public double angleBetween(Vector v) {
		return angleBetween(this, v);
	}

	/**
	 * angle between two vectors
	 * 
	 * @param a
	 *            first vector
	 * @param b
	 *            second vector
	 * @return angle between them
	 */
	public static double angleBetween(Vector a, Vector b) {
		double cosTheta = dot(a, b) / (a.getMagnitude() * b.getMagnitude());
		double angle = Math.toDegrees(Math.acos(cosTheta));
		angle = ResourceFunctions.putAngleInRange(angle);

		return Math.min(angle, 360 - angle);
	}

	/**
	 * angle: --, total: --, x: --, y: --
	 */
	public String toString() {
		return String.format("Angle: %f, Total: %f, X: %f, Y: %f", this.getAngle(), this.getMagnitude(), this.getX(),
				this.getY());
	}

	/**
	 * finds vector of magnitude 1
	 * 
	 * @param v
	 *            vector
	 * @return vector w/ magnitude 1
	 */
	public static Vector normalized(Vector v) {
		Vector newVec = new Vector(v);
		newVec.setTotal(1.0);
		return newVec;
	}

}
