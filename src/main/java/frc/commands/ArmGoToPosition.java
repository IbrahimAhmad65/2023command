package frc.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Arm;

public class ArmGoToPosition extends CommandBase {
    private Arm arm;
    private Translation2d targetPose;
    public ArmGoToPosition(Arm arm, Translation2d targetPose) {
        super();
        addRequirements(arm);
        this.arm = arm;
        this.targetPose = targetPose;
    }
    public ArmGoToPosition(Arm arm, double angle, double length) {
        super();
        addRequirements(arm);
        this.arm = arm;
        this.targetPose = new Translation2d(length* Math.cos(angle), length * Math.sin(angle));
    }

    @Override
    public void initialize() {
        arm.setTargetLength(targetPose.getNorm());
        arm.setTargetAngle(targetPose.getAngle().getRadians());
    }

    @Override
    public boolean isFinished() {
        System.out.println("ArmGoToPosition.isFinished(): " + arm.isAtTargetAngle() + " " + arm.isAtTargetLength());
        return arm.isAtTargetAngle() && arm.isAtTargetLength();
    }
}
