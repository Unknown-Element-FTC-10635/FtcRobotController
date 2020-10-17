package org.firstinspires.ftc.teamcode.Motors;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev = 103.6, gearing = 3.7, maxRPM = 1780, orientation = Rotation.CCW)
@DeviceProperties(xmlTag = "NeveRestOrbital3_7", name = "NeveRest Orbital 3.7", builtIn = true)
@DistributorInfo(distributor = "AndyMark", model = "am-3461a", url = "https://www.andymark.com/products/neverest-orbital-3-7-gearmotor")
@ExpansionHubPIDFVelocityParams(P = 2.0, I = 0.5, F = 11.1)
@ExpansionHubPIDFPositionParams(P = 5.0)
public interface NeveRestOrbital3_7 {
}