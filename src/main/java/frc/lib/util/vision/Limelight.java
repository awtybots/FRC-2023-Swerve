package frc.util.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.math.Vector2;

public class Limelight extends SubsystemBase {

    public final double mountingHeight;
    public final double mountingAngle;
    public final Vector2 offsetFromMechanism;

    private final NetworkTable netTable = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight(double mountingHeight, double mountingAngle, Vector2 offsetFromMechanism) {
        this.mountingAngle = mountingAngle;
        this.mountingHeight = mountingHeight;
        this.offsetFromMechanism = offsetFromMechanism;
    }

    /**
     * Get the relative displacement vector from the mechanism to the goal. The x axis is horizontal
     * displacement and the y axis is vertical displacement. Values are in meters.
     *
     * @return The displacement vector, or null if no vision target is detected.
     */
    public Vector2 getDisplacementFrom(VisionTarget target) {
        if (!hasVisibleTarget()) return null;

        double angleY = targetYOffset();

        double opposite = target.height - this.mountingHeight;
        double tangent = Math.tan(Math.toRadians(this.mountingAngle + angleY));
        double adjacent = opposite / tangent;

        return new Vector2(adjacent, target.goalHeight - this.mountingHeight)
                .minus(this.offsetFromMechanism);
    }

    public boolean hasVisibleTarget() {
        return getValue(TableEntry.HasValidTargets) == 1.0;
    }

    /**
     * Get the horizontal angle from the crosshair to the center of the target
     *
     * @return An angle between -29.8deg and 29.8deg
     */
    public double targetXOffset() {
        return getValue(TableEntry.TargetXOffset);
    }

    /**
     * Get the vertical angle from the crosshair to the center of the target
     *
     * @return An angle between -24.85deg and 24.85deg
     */
    public double targetYOffset() {
        return getValue(TableEntry.TargetYOffset);
    }

    /**
     * Set the current pipeline for vision processing
     *
     * @param pipeline integer from 0 to 9, inclusive
     */
    public void setPipeline(int pipeline) {
        if (0 >= pipeline && pipeline <= 9) setValue(TableEntry.CurrentPipeline, pipeline);
    }

    public void toggleLED(LEDMode state) {
        setValue(TableEntry.LEDMode, state.ordinal());
    }

    private double getValue(TableEntry entry) {
        NetworkTableValue v = netTable.getEntry(entry.getter).getValue();
        if (v.getType() == NetworkTableType.kDouble) {
            return v.getDouble();
        } else {
            return -1.0;
        }
    }

    private boolean setValue(TableEntry entry, double value) {
        if (entry.setter == "") return false;
        return netTable.getEntry(entry.setter).setDouble(value);
    }

    public enum LEDMode {
        /** Uses the default mode set in the active Pipeline */
        PipelineDefault,
        Off,
        Blinking,
        On
    }

    private enum TableEntry {
        HasValidTargets("tv"),
        TargetXOffset("tx"),
        TargetYOffset("ty"),
        TargetArea("ta"),
        TargetSkew("ts"),
        PipelineLatency("tl"),
        CurrentPipeline("getpipe", "pipeline"),
        OperationMode("camMode", "camMode"),
        LEDMode("LEDMode", "LEDMode");
        public String setter, getter;

        TableEntry(String getter, String setter) {
            this.setter = setter;
            this.getter = getter;
        }

        TableEntry(String getter) {
            this.setter = "";
            this.getter = getter;
        }
    }
}
