package frc.robot.subsystems.MechanicalParts;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonFX mIntakeMotor;
    private boolean kKeep;
    private double kIntakePct;

    public IntakeSubsystem() {
        mIntakeMotor = new WPI_TalonFX(Constants.ClawConstants.kIntakeMotorId);
        configMotors();
    }

    private void configMotors() {
        mIntakeMotor.configFactoryDefault();
        mIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mIntakeMotor.setNeutralMode(NeutralMode.Brake);
        mIntakeMotor.configPeakOutputForward(Constants.ClawConstants.kMaxPercentOutput);
        mIntakeMotor.configPeakOutputReverse(-Constants.ClawConstants.kMaxPercentOutput);
    }

    public void intake(double pct, boolean keep) {
        kKeep = keep;
        if (keep) {
            kIntakePct = pct * Constants.ClawConstants.kMaxPercentOutput;
        } else {
            mIntakeMotor.set(pct * Constants.ClawConstants.kMaxPercentOutput);
        }
    }

    public double getOutputCurrent() {
        return mIntakeMotor.getSupplyCurrent();
    }

    public void stopIntake() {
        mIntakeMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw Current", getOutputCurrent());
        if (kKeep) {
            mIntakeMotor.set(kIntakePct);
        }
    }
}
