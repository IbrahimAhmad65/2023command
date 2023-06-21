package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Wrist;
import teamtators.sim.WristSim;

public class WristToAngle extends CommandBase {
    private final Wrist wrist;
    private double angle;
    public WristToAngle(Wrist wrist, double angle) {
        super();
        addRequirements(wrist);
        this.wrist = wrist;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        wrist.setTargetAngle(angle);
    }


    @Override
    public boolean isFinished() {
        //System.out.println("Wrist angle: " + wrist.getDynamicsSim().getAngle() + " Target angle: " + angle);
        return wrist.isAtTargetAngle();
    }
}
