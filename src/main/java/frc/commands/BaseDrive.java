package frc.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.SwerveDrive;

import java.util.function.Consumer;

public class BaseDrive extends CommandBase implements Consumer<ChassisSpeeds> {
    private ChassisSpeeds chassisSpeeds;
    private SwerveDrive swerveDrive;

    public BaseDrive(SwerveDrive swerveDrive){
        super();
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }




    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void accept(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }
}
