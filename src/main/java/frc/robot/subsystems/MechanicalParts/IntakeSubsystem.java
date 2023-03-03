package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax mIntakeMotor;

    private CANSparkMax mLeftIntakeMotor;
    private CANSparkMax mRightIntakeMotor;

    private final RelativeEncoder mIntakeEncoder;

    private final RelativeEncoder mLeftIntakeEncoder;
    private final RelativeEncoder mRightIntakeEncoder;

    // private final SparkMaxPIDController mLeftIntakePIDController;
    // private final SparkMaxPIDController mRightIntakePIDController;

    private final CANSparkMax[] intakeMotors;
    // private final RelativeEncoder[] intakeEncoders;
    // private final SparkMaxPIDController[] pidControllers;

    public IntakeSubsystem() {
        mIntakeMotor = new CANSparkMax(0, MotorType.kBrushless);
        mIntakeMotor.restoreFactoryDefaults();

        mIntakeMotor.setSmartCurrentLimit(Constants.ClawConstants.kIntakeCurrentLimit);

        mIntakeEncoder = mIntakeMotor.getEncoder();


        mLeftIntakeMotor =
                new CANSparkMax(Constants.ClawConstants.kLeftIntakeMotorId, MotorType.kBrushless);
        mRightIntakeMotor =
                new CANSparkMax(Constants.ClawConstants.kRightIntakeMotorId, MotorType.kBrushless);

                
        mLeftIntakeMotor.restoreFactoryDefaults();
        mRightIntakeMotor.restoreFactoryDefaults();

        // Current limit
        mLeftIntakeMotor.setSmartCurrentLimit(Constants.ClawConstants.kIntakeCurrentLimit);
        mRightIntakeMotor.setSmartCurrentLimit(Constants.ClawConstants.kIntakeCurrentLimit);

        intakeMotors = new CANSparkMax[] {mLeftIntakeMotor, mRightIntakeMotor};

        mLeftIntakeEncoder = mLeftIntakeMotor.getEncoder();
        mRightIntakeEncoder = mRightIntakeMotor.getEncoder();
        // intakeEncoders = new RelativeEncoder[] {mLeftIntakeEncoder, mRightIntakeEncoder};

        // mLeftIntakePIDController = mLeftIntakeMotor.getPIDController();
        // mRightIntakePIDController = mRightIntakeMotor.getPIDController();
        // pidControllers = new SparkMaxPIDController[]{mLeftIntakePIDController,
        // mRightIntakePIDController};

        // mLeftIntakePIDController.setFeedbackDevice(mLeftIntakeEncoder);
        // mRightIntakePIDController.setFeedbackDevice(mRightIntakeEncoder);

    }

    public void intake(double pct) {
        double multiplier;
        if (pct > 0) {
            multiplier = 1;
        } else {
            multiplier = 1;
        }

        mIntakeMotor.set(pct * multiplier);

        intakeMotors[0].set(-pct * multiplier);
        intakeMotors[1].set(pct * multiplier);
    }

    public void intake(double pct, long time) {

        double multiplier;
        if (pct > 0) {
            multiplier = 1;
        } else {
            multiplier = 1;
        }

        mIntakeMotor.set(pct * multiplier);

        intakeMotors[0].set(-pct * multiplier);
        intakeMotors[1].set(pct * multiplier);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e1) {
            e1.printStackTrace();
        }
    }

    public void stopIntake() {
        mIntakeMotor.set(0);
        for (CANSparkMax motor : intakeMotors) motor.set(0);
    }
}
