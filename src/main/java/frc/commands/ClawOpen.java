package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Claw;

public class ClawOpen extends CommandBase {
    private final Claw claw;
    private boolean state;
    public ClawOpen(Claw claw, boolean state) {
        super();
        addRequirements(claw);
        this.claw = claw;
    }

    @Override
    public void initialize() {
        claw.setOpen(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
