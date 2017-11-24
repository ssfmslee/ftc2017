package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Bruno: Everything-V1", group="Linear Opmode")

public class BrunoEverythingV1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftGrip      = null;
    private Servo rightGrip     = null;
    private DcMotor Extend      = null;
    private DcMotor Chain       = null;
    private DcMotor Motor1      = null;
    private DcMotor Motor2      = null;
    private DcMotor Motor3      = null;
    private DcMotor Motor4      = null;
    private double left_pos     = 0.5;
    private double right_pos    = 0.5;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0.0;
    private double ExtendPower;
    private double ChainPower;
    private double pivotR;
    private double verticalDrv;
    private double horizontalDrv;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    
    //Hardware Map    
    leftGrip  = hardwareMap.get(Servo.class, "left_servo");
    rightGrip = hardwareMap.get(Servo.class, "right_servo");
    Extend    = hardwareMap.get(DcMotor.class, "MotorExtend");
    Chain     = hardwareMap.get(DcMotor.class, "MotorLift");
    Motor1    = hardwareMap.get(DcMotor.class, "Motor1");
    Motor2    = hardwareMap.get(DcMotor.class, "Motor2");
    Motor3    = hardwareMap.get(DcMotor.class, "Motor3");
    Motor4    = hardwareMap.get(DcMotor.class, "Motor4");
    
    //Setup
    leftGrip.scaleRange(MIN_POS,MAX_POS);
    rightGrip.scaleRange(MIN_POS,MAX_POS);
    leftGrip.setPosition(left_pos);
    rightGrip.setPosition(right_pos);
    Extend.setDirection(DcMotor.Direction.FORWARD);
    Chain.setDirection(DcMotor.Direction.FORWARD);
    Motor1.setDirection(DcMotor.Direction.FORWARD);
    Motor2.setDirection(DcMotor.Direction.FORWARD);
    Motor3.setDirection(DcMotor.Direction.FORWARD);
    Motor4.setDirection(DcMotor.Direction.FORWARD);
    
//Main
    while (opModeIsActive()) {
        
        //Grabber
        if (gamepad2.dpad_down){
            left_pos +=0.05;
            right_pos -=0.05;
            }
        if (gamepad2.dpad_up){
            left_pos -=0.05;
            right_pos +=0.05;
            }
        if (right_pos >=1.0){
            right_pos =0.99;
            }   
        if (left_pos <=0){
            left_pos =0.01;
            }
        if (left_pos >=1.0){
            left_pos =0.99;
            }   
        if (right_pos <=0){
            right_pos =0.01;
            }
            
        //Chain and Extend 
        ExtendPower  = gamepad2.left_stick_y ;
        ChainPower   = gamepad2.right_stick_y ;
    
        Extend.setPower(ExtendPower/3);
        Chain.setPower(ChainPower/3);
        
        //Drive/Pivot    
        pivotR  = -gamepad1.left_stick_x ;
        verticalDrv = -gamepad1.right_stick_y;
        horizontalDrv = gamepad1.right_stick_x;
        
        if((gamepad1.right_stick_y == 0.0) && (gamepad1.right_stick_x == 0.0)){
        Motor1.setPower(pivotR);
        Motor2.setPower(pivotR);
        Motor3.setPower(pivotR);
        Motor4.setPower(pivotR);
        }
        if(gamepad1.left_stick_x == 0.0){
        Motor2.setPower(verticalDrv);
        Motor4.setPower(-verticalDrv);
        Motor1.setPower(horizontalDrv);
        Motor3.setPower(-horizontalDrv); 
        }
        //End of Main
        }
    }
}
