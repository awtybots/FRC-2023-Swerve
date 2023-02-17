package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;

    private final RelativeEncoder mRightArmEncoder;

    private final SparkMaxPIDController mRightArmPIDController;

    public double armHeight;

    private final ElevatorSubsystem sElevator;

    public ArmSubsystem(ElevatorSubsystem sElevatorSubsystem) {
        sElevator = sElevatorSubsystem;

        mLeftArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmMotorId, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmMotorId, MotorType.kBrushless);
        // mLeftArmMotor.restoreFactoryDefaults();
        mRightArmMotor.restoreFactoryDefaults();
        mRightArmMotor.setInverted(true);
        mLeftArmMotor.follow(mRightArmMotor, true);

        armHeight = Constants.ArmConstants.initialHeight;

        mRightArmEncoder = mRightArmMotor.getEncoder();

        mRightArmPIDController = mRightArmMotor.getPIDController();
        ;

        mRightArmPIDController.setP(0.04);
        mRightArmPIDController.setI(0);
        mRightArmPIDController.setD(0);
        mRightArmPIDController.setOutputRange(-1, 1);

        // mRightArmPIDController.setFeedbackDevice(mRightArmEncoder);

    }

    public void setRotation(int value) {
        armHeight = value;
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(mRightArmEncoder.getPosition() / 360);
    }

    public double getAngle(double talon) {
        return talon * (Math.PI / 2) / 45;
    }

    public double getMaximumRotation() {
        double value =
                (Math.PI
                                - Math.acos(
                                        (sElevator.getDistance() - Constants.ElevatorConstants.ElevatorOffset)
                                                / Constants.ArmConstants.armLength)
                                + Constants.ArmConstants.startingAngle)
                        * 90
                        / Math.PI;
        if (Double.isNaN(value)) {
            value = Constants.ArmConstants.maximumHeight;
        }
        return value;
    }

    public void drive(double pct) {
        armHeight =
                MathUtil.clamp(armHeight, Constants.ArmConstants.minimumHeight, getMaximumRotation());
        armHeight += pct;

        SmartDashboard.putNumber("armHeight ", armHeight);
        SmartDashboard.putNumber("armEncoderReadout2 ", mRightArmEncoder.getPosition());

        mRightArmPIDController.setReference(armHeight, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm... Angle?", mRightArmEncoder.getPosition());
        SmartDashboard.putNumber("Calculated Maximum Angle", getMaximumRotation());
    }

    public void stop() {
        mRightArmMotor.set(0);
    }
}
