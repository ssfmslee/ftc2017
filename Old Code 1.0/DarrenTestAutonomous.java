/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous

public class DarrenTestAutonomous extends DarrenLinearDrive{
    private ColorSensor colorSensor1;
   private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MotorOne;
    private DcMotor MotorTwo;
    private DcMotor MotorThree;
    private DcMotor MotorFour;
    private DigitalChannel touchSensor;
    private DistanceSensor rangeSensor1;
    private OpticalDistanceSensor sensor_ods;
    
    
    public void main() throws InterruptedException{
        MotorOne = hardwareMap.dcMotor.get("Motor1");
        MotorTwo = hardwareMap.dcMotor.get("Motor2");
        MotorThree = hardwareMap.dcMotor.get("Motor3");
        MotorFour = hardwareMap.dcMotor.get("Motor4");
        */
       /* MotorOne.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        MotorTwo.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        MotorThree.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        MotorFour.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS); */
       /* 
        MotorOne.setDirection(DcMotor.Direction.FORWARD);
        MotorTwo.setDirection(DcMotor.Direction.FORWARD);
        MotorThree.setDirection(DcMotor.Direction.FORWARD);
        MotorFour.setDirection(DcMotor.Direction.FORWARD);
        
        while(runtime<=5){
        MotorTwo.setPower(1.0);
        MotorThree.setPower(1.0);
        }
        telemetry.addData("Status", "Run Time: " + runtime.toString());
           telemetry.addData("Motors", "left (%.2f), right (%.2f)", verticalDrv, horizontalDrv);
            telemetry.update();
    }
}
*/
