package me.nicholasnadeau.robot.kukalbr.phd;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.TimeUnit;

import me.nicholasnadeau.robot.kukalbr.utilities.LBRUtilities;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class PHDDaqPokeAutomated extends RoboticsAPIApplication {
	private Controller sunrise;
	private LBR lbr;
	private Tool tool;
	private double[] cartesianVelocities = new double[] { 50, 100, 150, 200, 250 }; // mm/s
	private double[] maxForces = new double[] { 5, 10, 20, 30 }; // N
	String[] collisionIDs = new String[] { "human", "table" };
	double currentCartVel;
	double currentMaxForce;
	String daqID;
	LIN upMotion;
	LIN downMotion;
	ForceCondition forceCondition;
	private Frame startFrame;
	private Frame endFrame;

	@Override
	public void initialize() {
		sunrise = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(sunrise, "LBR_iiwa_7_R800_1");
		tool = getApplicationData().createFromTemplate("RobotiqPointer");
	}

	void performPoke() {
		forceCondition = ForceCondition.createSpatialForceCondition(tool.getDefaultMotionFrame(), currentMaxForce);
		upMotion = lin(startFrame).setCartVelocity(currentCartVel);
		downMotion = lin(endFrame).setCartVelocity(currentCartVel).breakWhen(forceCondition);

		String timeStamp = new SimpleDateFormat("yyyyMMddHHmmss").format(new Date());
		String logFilename = String.format("%s-s%s-f%s-%s.log", daqID, currentCartVel, currentMaxForce,
				timeStamp);
		long timeout = 30; // s
		int sampleRate = 1; // ms
		DataRecorder dataRecorder = new DataRecorder(logFilename, timeout, TimeUnit.SECONDS, sampleRate);
		dataRecorder.addInternalJointTorque(lbr);
		dataRecorder.addExternalJointTorque(lbr);
		dataRecorder.addCartesianForce(tool.getDefaultMotionFrame(), null);
		dataRecorder.addCartesianTorque(tool.getDefaultMotionFrame(), null);
		dataRecorder.addCommandedJointPosition(lbr, AngleUnit.Degree);
		dataRecorder.addCurrentJointPosition(lbr, AngleUnit.Degree);
		dataRecorder.addCommandedCartesianPositionXYZ(tool.getDefaultMotionFrame(), World.Current.getRootFrame());
		dataRecorder.addCurrentCartesianPositionXYZ(tool.getDefaultMotionFrame(), World.Current.getRootFrame());
		dataRecorder.enable();

		// move
		getLogger().info("Moving down");
		dataRecorder.startRecording();
		IMotionContainer downMotionContrainer = tool.move(downMotion);
		if (downMotionContrainer.getFiredBreakConditionInfo() != null) {
			getLogger().info("Force condition fired");
		}
		tool.move(upMotion);
		dataRecorder.stopRecording();

		getLogger().info(String.format("DAQ log:\t%s", dataRecorder.getFileName()));
	}

	@Override
	public void run() {
		// load tool
		tool.attachTo(lbr.getFlange());
		getLogger().info("Tool data loaded");

		// move to forward starting pose
		getLogger().info("Moving to forward start position");
		tool.move(LBRUtilities.ptpToHandGuidanceStart.setJointVelocityRel(LBRUtilities.SAFE_REL_JOINT_VELOCITY));

		startFrame = lbr.getCurrentCartesianPosition(tool.getDefaultMotionFrame());
		startFrame.setZ(200);
		endFrame = startFrame.copy();
		endFrame.setZ(-25);

		tool.move(ptp(startFrame).setJointVelocityRel(LBRUtilities.SAFE_REL_JOINT_VELOCITY));

		// ask collision type
		int collisionIdx = getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION,
				"Select collision ID.", collisionIDs[0], collisionIDs[1]);
		daqID = collisionIDs[collisionIdx];

		for (int cartVelIdx = 0; cartVelIdx < cartesianVelocities.length; cartVelIdx++) {
			currentCartVel = cartesianVelocities[cartVelIdx];
			getLogger().info(String.format("Cartesian velocity:\t%s", currentCartVel));

			for (int forceIdx = 0; forceIdx < maxForces.length; forceIdx++) {
				currentMaxForce = maxForces[forceIdx];
				getLogger().info(String.format("Max force:\t%s", currentMaxForce));

				for (int i = 0; i < 10; i++) {
					getLogger().info(String.format("Starting iteration:\t%d", i));
					performPoke();
				}
			}
		}

		getLogger().info("App finished");
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		PHDDaqPokeAutomated app = new PHDDaqPokeAutomated();
		app.runApplication();
	}
}
