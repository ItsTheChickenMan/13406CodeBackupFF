/*
Copyright 2022 FIRST Tech Challenge Team 13406

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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public class SwivelTest extends LinearOpMode {
    public DcMotorEx swivelMotor;

    @Override
    public void runOpMode() {
        swivelMotor = hardwareMap.get(DcMotorEx.class, "swivelMotor");
        
        swivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        swivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        swivelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        double position = 0;
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double movementSwivel = Range.clip(-gamepad1.left_stick_y, -1, 1);
            
            telemetry.addData("movementSwivel", movementSwivel);
            telemetry.addData("swivelMotor target", swivelMotor.getTargetPosition());
            telemetry.addData("swivelMotor current", swivelMotor.getCurrentPosition());
            telemetry.update();
            
            swivelMotor.setPower(movementSwivel);
            
            if(movementSwivel != 0.0){
                //position += movementSwivel * 2;
                
                //swivelMotor.setTargetPosition((int)position);
                
                //swivelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                //swivelMotor.setPower(movementSwivel);
            }
        }
    }
}
