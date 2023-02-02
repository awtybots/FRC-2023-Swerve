package frc.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.HashMap;

public class AutonManager {
    private final HashMap<String, Command> autonSequences;
    private final SendableChooser<String> dashboardSelector;

    private Command defaultCommand = null;

    private final String autonSelectorKey = "AutonChooser";

    public AutonManager() {
        autonSequences = new HashMap<String, Command>();
        dashboardSelector = new SendableChooser<String>();
        defaultCommand = null;
    }

    private void addOption(String name, Command seq, boolean isDefault) {
        if (name == null) {
            throw new IllegalArgumentException("Must provide a non-null autonomous name");
        }
        autonSequences.put(name, seq);

        if (isDefault) {
            defaultCommand = seq;
            dashboardSelector.setDefaultOption(name, name);
        } else {
            dashboardSelector.addOption(name, name);
        }
    }

    public void addOption(String name, Command seq) {
        addOption(name, seq, false);
    }

    public void addDefaultOption(String name, Command seq) {
        addOption(name, seq, true);
    }

    public Command getSelected() {
        final String dashboardSelection =
                NetworkTableInstance.getDefault()
                        .getTable("SmartDashboard")
                        .getSubTable(autonSelectorKey)
                        .getEntry("active")
                        .getString("_");

        if (dashboardSelection == "_") {
            System.out.printf(
                    "No auton retrieved from NetworkTablesEntry 'SmartDashboard/%s/active'\n",
                    autonSelectorKey);

            if (defaultCommand != null) {
                System.out.printf("Running default autonomous: '%s'", defaultCommand.getName());
                return defaultCommand;
            } else {
                System.out.println("No default command set, doing nothing for autonomous");
                return new InstantCommand();
            }
        } else {
            System.out.printf(
                    "Retrieved auton selection: '%s' from NetworkTables key '/SmartDashboard/%s/active'\n",
                    dashboardSelection, autonSelectorKey);

            final Command selectedCommand = autonSequences.get(dashboardSelection);

            if (selectedCommand != null) {
                System.out.printf("Running '%s' for autonomous\n", selectedCommand.getName());
                return selectedCommand;
            } else {
                // Somehow NetworkTables has returned a value that was not in the list of
                // options we gave it. Either that or we failed to remember the options we gave it
                System.out.printf("Auton selection of `%s` was not found\n", dashboardSelection);

                if (defaultCommand != null) {
                    System.out.printf(
                            "Running default autonomous `%s`\n", defaultCommand.getName());
                    return defaultCommand;
                } else {
                    System.out.println(
                            "No default autonomous was selected. Doing nothing for autonomous.");
                    return new InstantCommand();
                }
            }
        }
    }

    public void displayChoices() {
        SmartDashboard.putData(autonSelectorKey, dashboardSelector);
    }
}
