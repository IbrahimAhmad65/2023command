package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimWriter extends SubsystemBase {

    public static boolean sim = true;
    private final Wrist wrist;
    private final Arm arm;
    private final Claw claw;

    public SimWriter(Arm arm, Wrist wrist, Claw claw) {
        super();
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
    }

    @Override
    public void periodic() {
        double armExtensionX = arm.getDynamicsSim().getLength() * Math.cos(arm.getDynamicsSim().getAngleRads());
        double armExtensionY = arm.getDynamicsSim().getLength() * Math.sin(arm.getDynamicsSim().getAngleRads());
        double wristAngle = wrist.getDynamicsSim().getAngle();
        boolean clawOpen = claw.isOpen();

        SmartDashboard.putNumber("Arm Extension X", armExtensionX);
        SmartDashboard.putNumber("Arm Extension Y", armExtensionY);
        SmartDashboard.putNumber("Wrist Angle", wristAngle);
        SmartDashboard.putBoolean("Claw State", clawOpen);
    }
}
