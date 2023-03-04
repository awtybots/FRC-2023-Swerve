package frc.robot.subsystems.MechanicalParts;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonFX mIntakeMotor;
    private double IntakePct;

    public IntakeSubsystem() {
        IntakePct = 0;
        mIntakeMotor = new WPI_TalonFX(Constants.ElevatorConstants.kLeftElevatorMotorId);
        configMotors();
    }

    private void configMotors() {
        mIntakeMotor.configFactoryDefault();
        mIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mIntakeMotor.setNeutralMode(NeutralMode.Brake);
        mIntakeMotor.configPeakOutputForward(Constants.ClawConstants.kMaxPercentOutput);
        mIntakeMotor.configPeakOutputReverse(-Constants.ClawConstants.kMaxPercentOutput);
    }

    public void intake(double pct) {
        IntakePct+= pct/10;
        IntakePct =
                MathUtil.clamp(
                        IntakePct,
                        -Constants.ClawConstants.kMaxPercentOutput,
                        Constants.ClawConstants.kMaxPercentOutput);
    }

    public void intake(double pct, long time) {
        IntakePct+= pct/10;
        IntakePct =
                MathUtil.clamp(
                        IntakePct,
                        -Constants.ClawConstants.kMaxPercentOutput,
                        Constants.ClawConstants.kMaxPercentOutput);

        try {
            Thread.sleep(time);
        } catch (InterruptedException e1) {
            e1.printStackTrace();
        }
    }

    public void stopIntake() {
        mIntakeMotor.set(0);
    }

    @Override
    public void periodic() {
        mIntakeMotor.set(IntakePct);
    }
}
