package org.bitbuckets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

    DifferentialDrivetrainSim sim;

    static float TICK_LENGTH_MS = 20;
    static float TICK_LENGTH_SECONDS = TICK_LENGTH_MS / 100;

    //final variables [correct]
    DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
            DCMotor.getNeo550(1),
            150,
            30,
            5,
            1,
            0.5,
    );
    final XboxController driverController = new XboxController(0);

    //state variables

    int lastRightVoltageCommand_volts = 0f;
    int lastRightVoltageCommand_volts = 0f;

    public void robotInit() {

    }

    public void robotPeriodic() {

        sim.setInputs(lastRightVoltageCommand_volts, lastRightVoltageCommand_volts);
        sim.update(TICK_LENGTH_SECONDS);

        Pose2d estimatedPose = sim.getPose();
        double[] poseArray = new double[] {estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation()};

        SmartDashboard.getEntry("robot-pose").setDoubleArray(poseArray);
    }



    public void teleopPeriodic() {
        lastRightVoltageCommand_volts = driverController.getRawAxis(1) * 12;
        lastRightVoltageCommand_volts = driverController.getRawAxis(0) * 12;
    }



}
