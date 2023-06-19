package frc.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.OperatorInterfaceConstants;
import frc.subsystems.OperatorInterface.JoystickData;
import teamtators.util.TatorCANPID;
import teamtators.util.TatorPigeon;

public class OperatorInterface extends SubsystemBase implements Supplier<JoystickData> {

    private JoystickData joystickData;
    private XboxController driver;
    private XboxController gunner;
    private TatorPigeon pigeon;


    public OperatorInterface(TatorPigeon tatorPigeon){
        this.pigeon = tatorPigeon;
        driver = new XboxController(OperatorInterfaceConstants.kDriverControllerPort);
        gunner = new XboxController(OperatorInterfaceConstants.kGunnerControllerPort);
        joystickData = new JoystickData();
    }
    @Override
    public void periodic() {

    }

    @Override
    public JoystickData get() {
        return joystickData;
    }

    public ChassisSpeeds getFieldCentricChassisSpeeds(){
        double xScalar = 1;
        double yScalar = 1;
        double rotationScalar = 1;
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
        fieldRelativeSpeeds.vxMetersPerSecond = joystickData.x1 * xScalar;
        fieldRelativeSpeeds.vyMetersPerSecond = joystickData.y1 * yScalar;
        fieldRelativeSpeeds.omegaRadiansPerSecond = joystickData.x2 * yScalar;

        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, new Rotation2d(Math.toRadians(pigeon.getYawContinuous())));
    }

    static class JoystickData{
        public double x1;
        public double x2;
        public double y1;
        public double y2;

    }
}
