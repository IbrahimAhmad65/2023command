package teamtators.sim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;

import java.io.FileWriter;

public class PinkarmSim {


    public static class ArmConfig {
        // 1 is the base arm
        // 2 is the telescoping arm
        public static double mass1 = 1;
        public static double mass2 = .8;
        public static double length1 = .8;
        public static double length2 = 1;
        public static double motorRotationsPerArmRotation = 100;

    }

    private VariableLengthArmSim armRotationSim;
    private ExtensionSim armExtensionSim;
    private double j;
    private double extension = 0;


    public PinkarmSim() {
        j = 1.0 / 3.0 * (ArmConfig.mass1 * ArmConfig.length1 * ArmConfig.length1) + 1.0 / 3.0 * (ArmConfig.mass2 * ArmConfig.length2 * ArmConfig.length2);
        //System.out.println(j);
        armRotationSim = new VariableLengthArmSim(DCMotor.getNEO(1), ArmConfig.motorRotationsPerArmRotation, j, ArmConfig.length1, -Math.PI, Math.PI / 2, ArmConfig.mass1 + ArmConfig.mass2, false);

        Matrix<N2, N1> x = VecBuilder.fill(-Math.PI/2,0);
        armRotationSim.setState(x);

        armExtensionSim = new ExtensionSim(DCMotor.getNeo550(1), ArmConfig.mass2, ArmConfig.length1, ArmConfig.length1 + ArmConfig.length2, 0, false);
    }

    public void periodic() {
        double numberOfIterations = 1;
        for (int i = 0; i < numberOfIterations; i++) {
            iterate(.02/numberOfIterations);
        }

        double fullLength = armExtensionSim.getTotalLength();

    }

    public String getDataToWrite(){
        return armExtensionSim.getTotalLength() * Math.cos(armRotationSim.getAngleRads()) + ", " + armExtensionSim.getTotalLength() * Math.sin(armRotationSim.getAngleRads()) + " ,";
    }

    private void iterate (double dt){
        extension = armExtensionSim.getExtension();
        j = 1.0 / 3.0 * (ArmConfig.mass1 * ArmConfig.length1 * ArmConfig.length1) + 1.0 / 3.0 * (ArmConfig.mass2 * ArmConfig.length2 * ArmConfig.length2) + ArmConfig.mass2 * extension * extension;
        armRotationSim.setCGRadius((ArmConfig.mass1 * ArmConfig.length1 / 2.0) + (ArmConfig.mass2) * (extension + ArmConfig.length2 / 2.0));
        armRotationSim.setMOI(j);

        armRotationSim.update(dt);
        armExtensionSim.update(dt);
    }

    public void setArmRotationVoltage(double voltage) {
        armRotationSim.setInputVoltage(voltage);
    }
    public void setArmExtensionVoltage(double voltage) {
        armExtensionSim.setInputVoltage(voltage);
    }

    public void setVoltages(double angularVoltage, double rotationalVoltage) {
        armRotationSim.setInputVoltage(angularVoltage);
        armExtensionSim.setInputVoltage(rotationalVoltage);
    }

    public void setVoltages(double[] voltages){
        setVoltages(voltages[0], voltages[1]);
    }

    public double getLength() {
        return armExtensionSim.getTotalLength();
    }

    public double getExtension() {
        return armExtensionSim.getExtension();
    }

    public double getAngleRads() {
        return armRotationSim.getAngleRads();
    }

    public double getExtensionAccel() {
        return armExtensionSim.getAccel();
    }
}
