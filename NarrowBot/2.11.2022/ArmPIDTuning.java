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
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

import AutoTools.EzrasLaw;

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

public class ArmPIDTuning extends EzrasLaw {
    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        
        globalTime = new ElapsedTime();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        int hubLevel = 0;
        
        // pidf
        PIDFCoefficients defaultpidf = armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients pidf = new PIDFCoefficients(defaultpidf.p, defaultpidf.i, 0, 0, MotorControlAlgorithm.LegacyPID);
        double p = pidf.p;
        double i = pidf.i;
        double d = pidf.d;
        double f = pidf.f;
        
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);
        
        // delta vals
        double lastTime = this.globalTime.seconds();
        double delta = 0;
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("PIDF", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString() );
            telemetry.addData("Hub Level", hubLevel);
            telemetry.addData("Target", armMotor.getTargetPosition());
            telemetry.addData("Encoder:", armMotor.getCurrentPosition());
            telemetry.addData("Velocity", armMotor.getVelocity());
            telemetry.addData("At target?", !armMotor.isBusy());
            telemetry.addLine("Gamepad1 left stick for P");
            telemetry.addLine("Gamepad1 right stick for i");
            telemetry.addLine("Gamepad2 right stick for d");
            telemetry.update();
            
            // get delta
            double time = this.globalTime.seconds();
            delta = time - lastTime;
            lastTime = time;
            
            // phoenix controls
            double movementArm = Range.clip(gamepad2.left_stick_y, -1, 1);
        
            // move claw to default positions
            if(movementArm != 0){
                // only move if arm is not currently moving (to prevent buildup)
                if(!armMotor.isBusy()){
                    // move arm 
                    if(movementArm > 0) hubLevel = Math.max(0, --hubLevel);
                    if(movementArm < 0) hubLevel = Math.min(2, ++hubLevel);
                }
            }
            
            this.armToHubLevel(hubLevel, 0.4);
            
            // change pidf
            if(gamepad1.left_stick_y != 0){
                double dif = -gamepad1.left_stick_y * delta;
                
                p += dif;
            }
            if(gamepad1.right_stick_y != 0){
                double dif = -gamepad1.right_stick_y * delta;
                
                i += dif;
            }
            if(gamepad2.right_stick_y != 0){
                double dif = -gamepad2.right_stick_y * delta;
                
                d += dif*0.5;
            }
            
            if(gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0 || gamepad2.right_stick_y != 0){
                try {
                    armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.LegacyPID));
                } catch(java.lang.UnsupportedOperationException e){
                    telemetry.addLine(e.toString());
                    telemetry.update();
                    break;
                }
            }
        }
        
        // continue to display any errors
        while(opModeIsActive());
    }
    
    public void armToHubLevel(int hubLevel, double power){
        armMotor.setTargetPosition(armPositions[hubLevel+1]);
        armMotor.setPower(power);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
