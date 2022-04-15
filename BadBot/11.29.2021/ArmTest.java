/*
Copyright 2021 FIRST Tech Challenge Team FTC_13406

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class ArmTest extends LinearOpMode {
    private Blinker control_Hub;
    private Gyroscope imu;
    private DcMotorEx motor1;
    private Servo servo1;
    private Servo servo2;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        telemetry.addData("Status", "Running");
        telemetry.update();
        
        motor1.setTargetPosition(-85);
        
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setVelocity(150);
        
        while(opModeIsActive()){
            telemetry.addData("Target", motor1.getTargetPosition());
            telemetry.addData("Encoder:", motor1.getCurrentPosition());
            telemetry.addData("Velocity", motor1.getVelocity());
            telemetry.addData("At target?", !motor1.isBusy());
            telemetry.update();
        }
    }
    
    /*public void goToRange(DcMotor motor, int targetLower, int targetUpper, double maxPower){
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        while(motor.getCurrentPosition() < targetLower && opModeIsActive()){
            telemetry.addData("pos:", motor.getCurrentPosition());
            telemetry.addData("target lower", targetLower);
            telemetry.update();
            
            //motor1.setVelocity(maxPower);
        }
        
        while(motor1.getCurrentPosition() > targetUpper && opModeIsActive()){
            telemetry.addData("pos:", motor.getCurrentPosition());
            telemetry.addData("target upper", targetLower);
            telemetry.update();
            //motor1.setVelocity(-maxPower);
        }
        
    }
    
    public void goToPositionUsingVelocity(DcMotorEx motor, int targetPosition, int targetVelocity){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // set motor velocity based on rate of change in position (effectively accounting for gravity)
        
        double lastPosition = motor.getCurrentPosition();
        while(motor.getCurrentPosition() != targetPosition && opModeIsActive()){
            // get rate
            double rateOfChange = motor.getCurrentPosition() - lastPosition;
            lastPosition = motor.getCurrentPosition();
            
            telemetry.addData("Rate of Change", rateOfChange);
            telemetry.addData("Target Velocity", targetVelocity);
            
            // faster rate of change = smaller factor
            // slower rate of change = greater factor
            double factor = (double)targetVelocity / (double)(targetVelocity + rateOfChange);
            
            telemetry.addData("Factor", factor);
            telemetry.update();
            
            motor.setVelocity(targetVelocity*factor);
        }
    }*/
}
