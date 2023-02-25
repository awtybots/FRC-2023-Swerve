package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {

    private CANSparkMax mPivotMotor;

    private final RelativeEncoder mPivotEncoder;

    private final SparkMaxPIDController mPivotPIDController;
    public double wristHeight;

    public ClawSubsystem() {
        wristHeight = Constants.ClawConstants.initialHeight;
        mPivotMotor = new CANSparkMax(Constants.ClawConstants.kPivotMotorId, MotorType.kBrushless);
        mPivotMotor.restoreFactoryDefaults();

        mPivotMotor.setSmartCurrentLimit(Constants.ClawConstants.kClawCurrentLimit);

        mPivotPIDController = mPivotMotor.getPIDController();

        mPivotEncoder = mPivotMotor.getEncoder();

        mPivotPIDController.setP(Constants.ClawConstants.kP);
        mPivotPIDController.setI(Constants.ClawConstants.kI);
        mPivotPIDController.setD(Constants.ClawConstants.kD);
        mPivotPIDController.setOutputRange(-0.2, 0.2);

        // mPivotPIDController.setFeedbackDevice(mPivotEncoder);
    }

    public void setRotation(int value) {
        wristHeight = value;
    }

    public void driveClaw(double pct) {
        wristHeight =
                MathUtil.clamp(
                        wristHeight,
                        Constants.ClawConstants.minimumHeight,
                        Constants.ClawConstants.maximumHeight);
        SmartDashboard.putNumber("wristHeight ", wristHeight);
        SmartDashboard.putNumber("wristEncoderReadout1 ", mPivotEncoder.getPosition());
        mPivotPIDController.setReference(wristHeight, CANSparkMax.ControlType.kPosition);
        wristHeight += pct / 3.0;
    }

    public void stopClaw() {
        mPivotMotor.set(0);
    }
}
