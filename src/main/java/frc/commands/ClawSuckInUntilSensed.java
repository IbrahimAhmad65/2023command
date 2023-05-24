package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Claw;
import frc.subsystems.SimWriter;

public class ClawSuckInUntilSensed extends CommandBase {
    private boolean sim = SimWriter.sim;
    private double power;
    private Claw claw;
    public ClawSuckInUntilSensed(Claw claw, double power) {
        super();
        addRequirements(claw);
        this.power = power;
        this.claw = claw;
    }

    @Override
    public void initialize() {
        claw.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return claw.isSensed();
    }
}
