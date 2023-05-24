package frc.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final boolean sim = SimWriter.sim;
    private boolean open = false;

    private double power = 0;
    private DigitalInput sensor;
    public Claw() {
        super();
        if (!sim) {
            sensor = new DigitalInput(0);
        }
    }

    public String getDataToWrite(){
        return (open ? "open" : "closed") + ",";
    }

    public void setOpen(boolean open) {
        this.open = open;
    }

    public void setPower(double power) {
        this.power = power;
    }
    public boolean isSensed() {
        if (sim) {
            return true;
        }
        return sensor.get();
    }

}
