package frc.util.math;

import java.util.function.BiFunction;
import java.util.function.Function;

public class Vector2 {
    public double x, y;

    public Vector2() {
        this(0, 0);
    }

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2(Vector2 copy) {
        this(copy.x, copy.y);
    }

    /**
     * Construct a new Vector2 instance from polar coordinates.
     *
     * @param magnitude The magnitude.
     * @param angle The angle measured from the X+ axis, in degrees.
     * @return The constructed Vector2 instance.
     */
    public static Vector2 fromPolar(double magnitude, double angle) {
        double x = magnitude * Math.cos(Math.toRadians(angle));
        double y = magnitude * Math.sin(Math.toRadians(angle));
        return new Vector2(x, y);
    }

    /** @return The magnitude. */
    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * The original Vector2 instance remains unchanged.
     *
     * @param magnitude The new magnitude.
     * @return The same Vector2 instance scaled to have the specified magnitude.
     */
    public Vector2 withMagnitude(double magnitude) { // * unused
        return scale(magnitude / getMagnitude());
    }

    /**
     * The original Vector2 instance is mutated.
     *
     * @param magnitude The new magnitude.
     * @return The same Vector2 instance scaled to have the specified magnitude.
     */
    public Vector2 setMagnitude(double magnitude) { // * unused
        return mutate(withMagnitude(magnitude));
    }

    /** @return The angle measured from the X+ axis, in degrees. */
    public double getAngle() {
        return Math.toDegrees(Math.atan2(y, x));
    }

    /**
     * The original Vector2 instance remains unchanged.
     *
     * @param angle The new angle from the X+ axis, in degrees.
     * @return A new Vector2 instance with the specified angle and the same magnitude.
     */
    public Vector2 withAngle(double angle) {
        return Vector2.fromPolar(getMagnitude(), angle);
    }

    /**
     * The original Vector2 instance is mutated.
     *
     * @param angle The new angle from the X+ axis, in degrees.
     * @return The same Vector2 instance with the specified angle and the same magnitude.
     */
    public Vector2 setAngle(double angle) { // * unused
        return mutate(withAngle(angle));
    }

    /**
     * The original Vector2 instances remain unchanged.
     *
     * @param otherVector The Vector2 to add to the original.
     * @return A new Vector2 instance with the result.
     */
    public Vector2 plus(Vector2 otherVector) {
        return binaryOperator(otherVector, (a, b) -> a + b);
    }

    /**
     * The original Vector2 instances remain unchanged.
     *
     * @param otherVector The Vector2 to subtract from the original.
     * @return A new Vector2 instance with the result.
     */
    public Vector2 minus(Vector2 otherVector) { // * unused
        return binaryOperator(otherVector, (a, b) -> a - b);
    }

    /**
     * The original Vector2 instance remains unchanged.
     *
     * @param s The factor to scale by.
     * @return A new Vector2 instance with the applied scale.
     */
    public Vector2 scaled(double s) {
        return unaryOperator(a -> a * s);
    }

    /**
     * The original Vector2 instance is mutated.
     *
     * @param s The factor to scale by.
     * @return The same Vector2 instance with the applied scale.
     */
    public Vector2 scale(double s) {
        return mutate(scaled(s));
    }

    /**
     * The original Vector2 instance remains unchanged.
     *
     * @return A new Vector2 instance with the applied inversion.
     */
    public Vector2 inverse() { // * unused
        return unaryOperator(a -> -a);
    }

    /**
     * The original Vector2 instance is mutated.
     *
     * @return The same Vector2 instance with the applied inversion.
     */
    public Vector2 invert() { // * unused
        return mutate(inverse());
    }

    @Override
    public Vector2 clone() {
        return new Vector2(this);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("(");
        sb.append(semiRound(x));
        sb.append(", ");
        sb.append(semiRound(y));
        sb.append(")");
        return sb.toString();
    }

    // private helper methods

    private Vector2 mutate(Vector2 result) {
        this.x = result.x;
        this.y = result.y;
        return result;
    }

    private Vector2 unaryOperator(Function<Double, Double> f) {
        return new Vector2(f.apply(this.x), f.apply(this.y));
    }

    private Vector2 binaryOperator(Vector2 otherVector, BiFunction<Double, Double, Double> f) {
        return new Vector2(f.apply(this.x, otherVector.x), f.apply(this.y, otherVector.y));
    }

    private double semiRound(double a) {
        return Math.round(a * 100.0) / 100.0;
    }
}
