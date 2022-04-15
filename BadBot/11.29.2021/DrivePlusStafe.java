/*
Copyright 2020 FIRST Tech Challenge Team FTC

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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;

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

    public class DrivePlusStafe extends LinearOpMode {
    private Blinker expansion_Hub;
    private Blinker control_Hub;
    private Gyroscope imu_1;
    private Gyroscope imu;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor intakeMotor;
    private DcMotor shooterMotor;
    private DcMotor clawMotor;
    private DcMotor guideMotor;
    private ColorSensor colorFloor;
    private CRServo clawServo;

    @Override
    public void runOpMode() {
        expansion_Hub = hardwareMap.get(Blinker.class, "Expansion Hub");
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        clawMotor = hardwareMap.get(DcMotor.class, "clawMotor");
        guideMotor = hardwareMap.get(DcMotor.class, "guideMotor");
        colorFloor = hardwareMap.get(ColorSensor.class, "colorFloor");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        guideMotor.setDirection(DcMotor.Direction.REVERSE);
        clawServo.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
       
        
        while (opModeIsActive()) {
        
        // Drive Train: 
        double leftBackPower;
        double leftFrontPower;
        double rightFrontPower;
        double rightBackPower;
        
        double movement_x;
        double movement_y;
        double movement_turn;
        
        movement_x = Range.clip(gamepad1.left_stick_x, -1, 1);
        movement_y = Range.clip(gamepad1.left_stick_y, -1, 1);
        movement_turn = Range.clip(gamepad1.right_stick_x, -1, 1);
        
        leftBackPower     = movement_y + movement_turn - movement_x;
        leftFrontPower    = movement_y + movement_turn + movement_x;
        rightFrontPower   = movement_y - movement_turn + movement_x;
        rightBackPower    = movement_y - movement_turn - movement_x;
        
        leftBack.setPower(leftBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        
        
        // Intake:    
        double intakeMotorPower;
        double intakeIn = 1;
        double intakeOut = -1;
        if(gamepad1.right_trigger>.01){
            intakeMotorPower = gamepad1.right_trigger;
        }
        else if (gamepad1.right_bumper){
            intakeMotorPower = intakeOut;
        }
        else {
            intakeMotorPower = 0;
        }
        intakeMotor.setPower(intakeMotorPower);
        
        //Shooter + Red Wheels:
        double shooterMotorPower;
        double shooterIn = 1;
        double shooterOut = -1;
        if(gamepad2.right_trigger>.01){
            shooterMotorPower = gamepad2.right_trigger;
        }
        else if (gamepad2.right_bumper){
            shooterMotorPower = shooterOut;
        }
        else {
            shooterMotorPower = 0;
        }
        shooterMotor.setPower(shooterMotorPower);
        
        double guideMotorPower;
        double guideIn = 1;
        double guideOut = -1;
        if(gamepad2.left_trigger>.01){
            guideMotorPower = gamepad2.left_trigger;
        }
        else if (gamepad2.left_bumper){
            guideMotorPower = guideOut;
        }
        else {
            guideMotorPower = 0;
        }
        guideMotor.setPower(guideMotorPower);
        
        //Claw:
        double clawMotorPower;
        clawMotorPower = gamepad2.right_stick_y;
        clawMotor.setPower(clawMotorPower);
        
      
        clawServo.setPower(-gamepad2.left_stick_x);
       
        
        
        
        
            
            // Telemetry:
            telemetry.addData("Target Left Back Motor Power", leftBackPower);
            telemetry.addData("Target Left Front Motor Power", leftFrontPower);
            telemetry.addData("Target Right Front Motor Power", rightFrontPower);
            telemetry.addData("Target Right Back Motor Power", rightBackPower);
            
            telemetry.addData("Right Back Motor Power", rightBack.getPower());
            telemetry.addData("Right Front Motor Power", rightFront.getPower());
            telemetry.addData("Left Front Motor Power", leftFront.getPower());
            telemetry.addData("Left Back Motor Power", leftBack.getPower());
            
            telemetry.addData("Intake Motor Power", intakeMotor.getPower());
            telemetry.addData("Target Intake Motor Power", intakeMotorPower);
            
            telemetry.addData("Target Shooter Motor Power", shooterMotorPower);
            telemetry.addData("Shooter Motor Power", shooterMotor.getPower());
            
            telemetry.addData("Target Claw Motor Power", clawMotorPower);
            telemetry.addData("Claw Motor Power", clawMotor.getPower());
            
            telemetry.addData("Target Guide Motor Power", guideMotorPower);
            telemetry.addData("Guide Motor Power", guideMotor.getPower());
            
            telemetry.addData("Claw Servo Power", clawServo.getPower());
            
            telemetry.addData("Blue", colorFloor.blue());
            telemetry.addData("Green", colorFloor.green());
            telemetry.addData("Red", colorFloor.red());
            
            telemetry.addData("Status", "Running");
            telemetry.update();
            
        }
    }
}
