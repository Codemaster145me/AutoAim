/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.DrivetrainSim;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;

import java.util.concurrent.RunnableScheduledFuture;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // 2020 High goal target height above ground
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(81.19);

    // Constants about how your camera is mounted to the robot
    public static final double CAMERA_PITCH_RADIANS =
            Units.degreesToRadians(15); // Angle "up" from horizontal
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24); // Height above floor

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(10);

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.5;
    final double LINEAR_D = 0.1;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.03;
    final double ANGULAR_D = 0.003;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    XboxController xboxController = new XboxController(0);

    // Drive motors
    PWMVictorSPX leftMotor = new PWMVictorSPX(0);
    PWMVictorSPX rightMotor = new PWMVictorSPX(1);
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    private double[] AprilTagPoseEstimate;

    // vision var
    double timer = 0;
    double SimTurn = 0;
    double FirstApriltag = 0;
    boolean check = false;

    @Override
    public void robotInit() {
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
    }

    @Override
    public void teleopPeriodic() {
        double forwardSpeed;
        double rotationSpeed;
        var result = camera.getLatestResult();

        SmartDashboard.putString("Best target number: ", "");
        SmartDashboard.putNumber("Sim turn: ", 0);
        SmartDashboard.putNumber("First April Tag", 0);
        SmartDashboard.putNumber("Timer: ", 0);

        if (xboxController.getRawAxis(4) > 0 && result.hasTargets()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            //var result = camera.getLatestResult();

            //FirstApriltag = result.getBestTarget().getYaw();

            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // (This forwardSpeed must be positive to go forward.)
                forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

                // Also calculate angular power
                // (This rotationSpeed must be positive to turn counter-clockwise.) 
                //rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
                //SimTurn = -turnController.calculate(result.getBestTarget().getYaw(), 0);
                if(SimTurn == -1){
                    SimTurn = 0;
                    FirstApriltag = 0;
                }
                else{
                    if(timer > 0){
                        SimTurn = -turnController.calculate(FirstApriltag, 0);
                        timer--;
                    }
                    else if(timer == 0){
                        FirstApriltag = result.getBestTarget().getYaw();
                        timer = 5;
                    }
                }

                SmartDashboard.putString("Best target number: ", result.getBestTarget().toString());
                SmartDashboard.putNumber("Sim turn: ", SimTurn);
                SmartDashboard.putNumber("First April Tag", FirstApriltag);
                SmartDashboard.putNumber("Timer: ", timer);

                //drive.arcadeDrive(forwardSpeed, SimTurn);
                drive.arcadeDrive(forwardSpeed, -SimTurn);

            } else {
                // If we have no targets, stay still.
                forwardSpeed = 0;
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            double speed = xboxController.getLeftY();
            double turn = xboxController.getRawAxis(2);

            double right = speed + turn;
            double left = speed - turn;

            drive.setDeadband(0.1);
            drive.arcadeDrive(-speed, -turn);

            /* 
            forwardSpeed = -xboxController.getLeftY();
            rotationSpeed = -xboxController.getRawAxis(2);
            */

        }

        // Use our forward/turn speeds to control the drivetrain
        //drive.arcadeDrive(forwardSpeed, rotationSpeed);
        SmartDashboard.updateValues();
    }

    ////////////////////////////////////////////////////
    // Simulation support

    DrivetrainSim dtSim;

    @Override
    public void simulationInit() {
        dtSim = new DrivetrainSim(leftMotor.getChannel(), rightMotor.getChannel(), camera);
    }

    @Override
    public void simulationPeriodic() {
        dtSim.update();
    }
}
