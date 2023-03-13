package frc.util.math;

public final class Convert {

    // #region encoders

    public static enum Encoder {
        RevRelativeEncoder(1.0),
        VersaPlanetaryIntegrated(1024.0),
        TalonFXIntegrated(2048.0),
        CTREMagEncoder(4096.0);

        public final double ticksPerRevolution;

        Encoder(double ticksPerRevolution) {
            this.ticksPerRevolution = ticksPerRevolution;
        }
    }

    public static double encoderPosToRevolutions(
            double sensorUnits, double gearRatio, Encoder encoder) {
        return sensorUnits / encoder.ticksPerRevolution * gearRatio;
    }

    public static double revolutionsToEncoderPos(
            double revolutions, double gearRatio, Encoder encoder) {
        return revolutions / gearRatio * encoder.ticksPerRevolution;
    }

    public static double encoderPosToAngle(double sensorUnits, double gearRatio, Encoder encoder) {
        return encoderPosToRevolutions(sensorUnits, gearRatio, encoder) * 360.0;
    }

    public static double angleToEncoderPos(double angle, double gearRatio, Encoder encoder) {
        return revolutionsToEncoderPos(angle / 360.0, gearRatio, encoder);
    }

    /**
     * @return distance (meters) that a winch would pull or wheel would travel
     */
    public static double encoderPosToDistance(
            double sensorUnits, double gearRatio, double diameter, Encoder encoder) {
        return encoderPosToRevolutions(sensorUnits, gearRatio, encoder) * (diameter * Math.PI);
    }

    public static double distanceToEncoderPos(
            double distance, double gearRatio, double diameter, Encoder encoder) {
        return revolutionsToEncoderPos(distance / (diameter * Math.PI), gearRatio, encoder);
    }

    public static double encoderVelToRevsPerSec(
            double sensorUnits, double gearRatio, Encoder encoder) {
        return sensorUnits * 10.0 / encoder.ticksPerRevolution * gearRatio;
    }

    public static double revsPerSecToEncoderVel(
            double revsPerSec, double gearRatio, Encoder encoder) {
        return revsPerSec / gearRatio * encoder.ticksPerRevolution / 10.0;
    }

    /**
     * @return encoder ticks per 100ms
     */
    public static double rpmToEncoderVel(double rpm, double gearRatio, Encoder encoder) {
        return revsPerSecToEncoderVel(rpm / 60.0, gearRatio, encoder);
    }

    public static double encoderVelToRpm(
            double sensorUnitsPer100ms, double gearRatio, Encoder encoder) {
        return encoderVelToRevsPerSec(sensorUnitsPer100ms, gearRatio, encoder) * 60.0;
    }

    /**
     * @return encoder ticks per 100ms
     */
    public static double angularVelToEncoderVel(double degPerSec, double gearRatio, Encoder encoder) {
        return revsPerSecToEncoderVel(degPerSec / 360.0, gearRatio, encoder);
    }

    public static double encoderVelToAngularVel(
            double sensorUnitsPer100ms, double gearRatio, Encoder encoder) {
        return encoderVelToRevsPerSec(sensorUnitsPer100ms, gearRatio, encoder) * 360.0;
    }

    /**
     * @return (encoder ticks per 100ms) per second
     */
    public static double angularAccelToEncoderAccel(
            double degPerSecPerSec, double gearRatio, Encoder encoder) {
        return revsPerSecToEncoderVel(degPerSecPerSec / 360.0, gearRatio, encoder);
    }

    public static double encoderVelToSpeed(
            double sensorUnitsPer100ms, double gearRatio, double diameter, Encoder encoder) {
        return encoderVelToRevsPerSec(sensorUnitsPer100ms, gearRatio, encoder) * (diameter * Math.PI);
    }

    public static double speedToEncoderVel(
            double speed, double gearRatio, double diameter, Encoder encoder) {
        return revsPerSecToEncoderVel(speed / (diameter * Math.PI), gearRatio, encoder);
    }

    public static double accelToEncoderAccel(
            double accel, double gearRatio, double diameter, Encoder encoder) {
        return speedToEncoderVel(accel, gearRatio, diameter, encoder);
    }

    // #endregion

    // #region units

    public static final double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static final double feetToMeters(double feet, double inches) {
        return inchesToMeters((feet * 12) + inches);
    }

    public static final double metersToInches(double meters) {
        return meters / 0.0254;
    }

    // #endregion

}
