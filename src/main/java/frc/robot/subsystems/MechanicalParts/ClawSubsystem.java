package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Claw;
import frc.robot.Constants.Presets;
import frc.robot.RobotContainer;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ClawSubsystem extends SubsystemBase {

    public double kWristGearRatio = 1.0 / 40.0;

    private CANSparkMax mPivotMotor;

    private final RelativeEncoder mPivotEncoder;

    private final SparkMaxPIDController mPivotPIDController;
    private double wristHeight;

    public ClawSubsystem() {
        wristHeight = Claw.initialHeight;
        mPivotMotor = new CANSparkMax(Claw.kPivotMotorId, MotorType.kBrushless);
        mPivotMotor.restoreFactoryDefaults();

        mPivotMotor.setInverted(true);

        mPivotMotor.setSmartCurrentLimit(Claw.kClawCurrentLimit);

        mPivotPIDController = mPivotMotor.getPIDController();

        mPivotEncoder = mPivotMotor.getEncoder();

        mPivotPIDController.setP(Claw.kP);
        mPivotPIDController.setI(Claw.kI);
        mPivotPIDController.setD(Claw.kD);
        mPivotPIDController.setOutputRange(-0.4, 0.4);

        // mPivotPIDController.setFeedbackDevice(mPivotEncoder);
    }

    public boolean isMotorConnected() {
        return mPivotMotor.getEncoder() != null;
    }

    public void setRotation(double value) {
        wristHeight = value;
    }

    public void setDegrees(double degrees) {
        double sensorUnits =
                Convert.angleToEncoderPos(degrees, kWristGearRatio, Encoder.RevRelativeEncoder);
        wristHeight = sensorUnits;
    }

    public double getAngle() {
        final double rawRevs = mPivotEncoder.getPosition();
        final double theta =
                Convert.encoderPosToAngle(rawRevs, kWristGearRatio, Encoder.RevRelativeEncoder);
        // System.out.println(theta);
        return theta + Claw.startingAngle;
    }

    public void resetEncoderValue() {
        wristHeight = 0;
        mPivotEncoder.setPosition(wristHeight);
    }

    private double positionError() {
        return mPivotEncoder.getPosition() - wristHeight;
    }

    public void driveClaw(double pct) {
        if (!RobotContainer.getResetPosMode()) {
            wristHeight = MathUtil.clamp(wristHeight, Claw.minimumHeight, Claw.maximumHeight);
        }
        wristHeight += (pct / 4.5) * Claw.clawConversion;
    }

    public boolean atTargetAngle() {
        return Math.abs(mPivotEncoder.getPosition() - wristHeight) < Presets.ClawThreshold;
    }

    @Override
    public void periodic() {
        mPivotPIDController.setReference(
                wristHeight,
                CANSparkMax.ControlType.kPosition,
                0,
                Claw.arbitraryFeedFowardRate * Math.cos(getAngle()));
        SmartDashboard.putNumber("Wrist encoder readout", mPivotEncoder.getPosition());
        SmartDashboard.putNumber("Wrist angle", getAngle() - Claw.startingAngle);
        SmartDashboard.putNumber("Wrist error", positionError());
    }

    public void stopClaw() {
        mPivotMotor.set(0);
    }
}
