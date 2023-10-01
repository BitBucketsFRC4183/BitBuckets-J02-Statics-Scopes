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


    static final double TICK_LENGTH_MS = 20d;
    static final double TICK_LENGTH_SECONDS = TICK_LENGTH_MS / 1000;

    OdometrySubsystem odometrySubsystem = new OdometrySubsystem();

    DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
            MOTOR,
            GEARING_ROT,
            MOI_JKG,
            MASS_KG,
            TRACK_WIDTH_M,
            WHEEL_RADIUS_M,
            STD_DEVS
    );
     static XboxController driverController = new XboxController(PORT);

    //state variables

    double lastRightVoltageCommand_volts = 0d;
    double lastLeftVoltageCommand_volts = 0d;

    public void robotInit() {
        driverController = new XboxController(PORT);
        OdometrySubsystem odometrySubsystem;
    }

    public void robotPeriodic() {

        sim.setInputs(lastRightVoltageCommand_volts, lastRightVoltageCommand_volts);
        sim.update(TICK_LENGTH_SECONDS);

        Pose2d estimatedPose = sim.getPose();
        double[] poseArray = CorrectCode.makePoseArray(
                estimatedPose.getX(),
                estimatedPose.getY(),
                estimatedPose.getRotation().getRadians()
        );

        CorrectCode.logPoseArray(poseArray);
    }



    public void teleopPeriodic() {
        lastRightVoltageCommand_volts = calculateDesiredVoltage(XboxController.Axis.kLeftY.value);
        lastLeftVoltageCommand_volts = calculateDesiredVoltage(XboxController.Axis.kRightY.value);
    }


    static double calculateDesiredVoltage(int axis) {
        return driverController.getRawAxis(axis) * 12 + GEARING_ROT;
    }






}
