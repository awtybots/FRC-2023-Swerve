package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ClawSubsystem extends SubsystemBase {

    public double kWristGearRatio = 1.0 / 40.0;

    private CANSparkMax mPivotMotor;

    public final RelativeEncoder mPivotEncoder;

    private final SparkMaxPIDController mPivotPIDController;
    public double wristHeight;

    public ClawSubsystem() {
        wristHeight = Constants.ClawConstants.initialHeight;
        mPivotMotor = new CANSparkMax(Constants.ClawConstants.kPivotMotorId, MotorType.kBrushless);
        mPivotMotor.restoreFactoryDefaults();

        mPivotMotor.setInverted(true);

        mPivotMotor.setSmartCurrentLimit(Constants.ClawConstants.kClawCurrentLimit);

        mPivotPIDController = mPivotMotor.getPIDController();

        mPivotEncoder = mPivotMotor.getEncoder();

        mPivotPIDController.setP(Constants.ClawConstants.kP);
        mPivotPIDController.setI(Constants.ClawConstants.kI);
        mPivotPIDController.setD(Constants.ClawConstants.kD);
        mPivotPIDController.setOutputRange(-0.2, 0.2);

        // mPivotPIDController.setFeedbackDevice(mPivotEncoder);
    }

    public void setRotation(double value) {
        wristHeight = value;
    }

    public double getAngle() {
        final double rawRevs = mPivotEncoder.getPosition();
        final double theta =
                Convert.encoderPosToAngle(rawRevs, kWristGearRatio, Encoder.RevRelativeEncoder);
        System.out.println(theta);
        return theta + Constants.ClawConstants.startingAngle;
    }

    public void resetEncoderValue() {
        wristHeight = 0;
        mPivotEncoder.setPosition(wristHeight);
    }

    public void driveClaw(double pct) {
        if (!RobotContainer.getResetPosMode()) {
            wristHeight =
                    MathUtil.clamp(
                            wristHeight,
                            Constants.ClawConstants.minimumHeight,
                            Constants.ClawConstants.maximumHeight);
        }
        wristHeight += (pct / 4.5) * Constants.ClawConstants.clawConversion;
    }

    public boolean isFinished() {
        return Math.abs(mPivotEncoder.getPosition() - wristHeight) < Constants.Position.ClawThreshold;
    }

    @Override
    public void periodic() {
        mPivotPIDController.setReference(wristHeight, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Wrist encoder readout", mPivotEncoder.getPosition());
        SmartDashboard.putNumber("Wrist angle", getAngle());
    }

    public void stopClaw() {
        mPivotMotor.set(0);
    }
}
