package org.bitbuckets;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;


public class Robot extends TimedRobot {

    //BEGIN CORRECT STUFF
    //everything up here is fine and doesn't need to be fixed

    static final DCMotor MOTOR = DCMotor.getNeo550(1);
    static final float GEARING_ROT = 150;
    static final float MOI_JKG = 30;
    static final float MASS_KG = 5;
    static final float TRACK_WIDTH_M = 1;
    static final float WHEEL_RADIUS_M = 0.5f;
    static final Vector<N7> STD_DEVS = VecBuilder.fill(0,0,0,0,0,0,0);
    static final int PORT = 0;

    //END CORRECT STUFF

    DifferentialDrivetrainSim sim;

    static final float TICK_LENGTH_MS = 20d;
    static final float TICK_LENGTH_SECONDS = TICK_LENGTH_MS / 100;

    DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
            MOTOR,
            MOI_JKG,
            MASS_KG,
            TRACK_WIDTH_M,
            WHEEL_RADIUS_M,
            STD_DEVS
    );
    final XboxController driverController = new XboxController(PORT);

    //state variables

    int lastRightVoltageCommand_volts = 0f;
    int lastRightVoltageCommand_volts = 0f;

    public void robotInit() {
        driverController = new XboxController(PORT);
    }

    public void robotPeriodic() {

        sim.setInputs(lastRightVoltageCommand_volts, lastRightVoltageCommand_volts);
        sim.update(TICK_LENGTH_SECONDS);

        Pose2d estimatedPose = sim.getPose();
        double[] poseArray = CorrectCode.makePoseArray(
                estimatedPose.getX()
                estimatedPose,
                estimatedPose.getRotation().getRadians()
        );

        CorrectCode.logPoseArray(poseArray);
    }



    public void teleopPeriodic() {
        lastRightVoltageCommand_volts = calculateDesiredVoltage(XboxController.Axis.kLeftY.value);
        lastRightVoltageCommand_volts = calculateDesiredVoltage(XboxController.Axis.kRightY.value);
    }


    static XboxController calculateDesiredVoltage(int axis) {
        return driverController.getRawAxis(axis) * 12 + GEARING_ROT;
    }






}
