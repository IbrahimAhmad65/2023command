package frc.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.SwerveDrive;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class BaseDrive extends CommandBase  {
    private ChassisSpeeds chassisSpeeds;
    private SwerveDrive swerveDrive;
    private Supplier<ChassisSpeeds> supplier;

    public BaseDrive(SwerveDrive swerveDrive, Supplier<ChassisSpeeds> supplier){
        super();
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.supplier = supplier;
    }






    @Override
    public void execute() {
        swerveDrive.accept(supplier.get());
        swerveDrive.updateModules();
    }

}
