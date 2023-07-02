package org.firstinspires.ftc.teamcode.drive.Systems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Config.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ArmRotatorPosition;
import org.firstinspires.ftc.teamcode.drive.ConstantValues.ClawRotatorPosition;

public class System {

    private SampleMecanumDrive robot;
    private String armRotatorPosition = "HALF";

    public System(SampleMecanumDrive sampleMecanumDrive){
        this.robot = sampleMecanumDrive;
    }

    public void next(){
        if(robot.equals(robot.armRotator.getPosition(), ArmRotatorPosition.DOWN,0.01)) {
            robot.armRotator.setPosition(ArmRotatorPosition.HALF);
            armRotatorPosition = "HALF";
            return;
        }

        if(robot.equals(robot.armRotator.getPosition(), ArmRotatorPosition.HALF, 0.01)){

            if(robot.equals(robot.clawRotator.getPosition(), ClawRotatorPosition.FACE_TOP, 0.01)){
                robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
                armRotatorPosition = "HIGH";
                return;
            }

            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_TOP);
            return;
        }

        robot.armRotator.setPosition(ArmRotatorPosition.LOW);
        armRotatorPosition = "LOW";
    }

    public void previous(){

        if(robot.equals(robot.armRotator.getPosition(), ArmRotatorPosition.LOW, 0.01)){
            robot.armRotator.setPosition(ArmRotatorPosition.HIGH);
            armRotatorPosition = "HIGH";
            return;
        }

        if(robot.equals(robot.armRotator.getPosition(), ArmRotatorPosition.HIGH, 0.01)){
            robot.armRotator.setPosition(ArmRotatorPosition.HALF);
            armRotatorPosition = "HALF";
            return;
        }

        if(robot.equals(robot.clawRotator.getPosition(), ClawRotatorPosition.FACE_TOP, 0.01)){
            robot.clawRotator.setPosition(ClawRotatorPosition.FACE_GROUND);
            return;
        }

        robot.armRotator.setPosition(ArmRotatorPosition.DOWN);
        armRotatorPosition = "DOWN";
    }

    public String getArmRotatorPosition(){
        return armRotatorPosition;
    }
}

