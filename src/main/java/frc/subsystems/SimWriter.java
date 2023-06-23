package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.FileWriter;

public class SimWriter extends SubsystemBase {

    public static boolean sim = false;
    private FileWriter writer;
    private final Wrist wrist;
    private final Arm arm;
    private final Claw claw;

    public SimWriter(Arm arm, Wrist wrist, Claw claw) {
        super();
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        if(sim){
        try {
            writer = new FileWriter("/home/ibrahim/simAll.txt");
        } catch (Exception e) {
            e.printStackTrace();
        }
        }
    }

    @Override
    public void periodic() {
        if(sim){

            try {
                writer.write(arm.getDynamicsSim().getDataToWrite() + wrist.getDynamicsSim().getDataToWrite() + claw.getDataToWrite()+ "\n");
                writer.flush();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
