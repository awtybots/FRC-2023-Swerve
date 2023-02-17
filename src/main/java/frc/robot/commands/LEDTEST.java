package frc.robot.commands;
import frc.robot.subsystems.ledutils;
import frc.robot.subsystems.ledutils.patterens_eneum;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDTEST extends CommandBase {

    private ledutils uLedutils;

    public LEDTEST(ledutils uLedutils){
        this.uLedutils = uLedutils;
    }

    @Override
    public void execute() {
        uLedutils.ivans_patterns(patterens_eneum.awtybots);
    }
    
}
