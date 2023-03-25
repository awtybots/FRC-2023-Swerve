package frc.robot.subsystems.MechanicalParts;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Claw;
import frc.robot.subsystems.LedSubsystem;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonFX mIntakeMotor;
    private boolean kKeep;
    private double kIntakePct;
    private final LedSubsystem s_Led;

    private double idlePct = 0.1;

    public IntakeSubsystem(LedSubsystem ledSubsystem) {
        s_Led = ledSubsystem;
        mIntakeMotor = new WPI_TalonFX(Claw.kIntakeMotorId);
        configMotors();
    }

    private void configMotors() {
        mIntakeMotor.configFactoryDefault();
        mIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mIntakeMotor.setNeutralMode(NeutralMode.Brake);
        mIntakeMotor.configPeakOutputForward(Claw.kMaxPercentOutput);
        mIntakeMotor.configPeakOutputReverse(-Claw.kMaxPercentOutput);
    }

    public void intake(double pct, boolean keep) {
        kKeep = keep;
        if (keep) {
            kIntakePct = pct * Claw.kMaxPercentOutput;
        } else {
            mIntakeMotor.set(pct * Claw.kMaxPercentOutput);
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
        if(getOutputCurrent() > 10) {
            if(RobotContainer.getIsCone()){
                s_Led.setAnimation("intakeCUBE", false);
                s_Led.setAnimation("intakeCONE", true);
            } else {
                s_Led.setAnimation("intakeCUBE", true);
                s_Led.setAnimation("intakeCONE", false);
            }
        } else {
            s_Led.setAnimation("intakeCUBE", false);
            s_Led.setAnimation("intakeCONE", false);
        }
        SmartDashboard.putNumber("Claw Current", getOutputCurrent());
        if (kKeep) {
            mIntakeMotor.set(kIntakePct);
        } else {
            if(RobotContainer.getIsCone()) {
                mIntakeMotor.set(-idlePct);
            } else {
                mIntakeMotor.set(idlePct);
            }
        }
    }
}
