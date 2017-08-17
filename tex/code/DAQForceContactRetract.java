package me.nicholasnadeau.robot.kukalbr.daq;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import me.nicholasnadeau.robot.kukalbr.utilities.LBRUtilities;
import me.nicholasnadeau.robot.kukalbr.utilities.RobotPacketGenerator;
import me.nicholasnadeau.robot.server.Server;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;

public class DAQForceContactRetract extends RoboticsAPIApplication {
    private Controller sunriseController;
    private LBR lbr;
    private Tool tool;
    private double maxForce = 10;
    private double cartVel = LBRUtilities.SAFE_ABS_LIN_VELOCITY;
    private int packetId = 0;

    /**
     * Auto-generated method stub. Do not modify the contents of this method.
     */
    public static void main(String[] args) {
        DAQForceContactRetract app = new DAQForceContactRetract();
        app.runApplication();
    }

    public void initialize() {
        sunriseController = getController("KUKA_Sunrise_Cabinet_1");
        lbr = (LBR) getDevice(sunriseController, "LBR_iiwa_7_R800_1");
        tool = getApplicationData().createFromTemplate("RobotiqPointer");
    }

    public void run() {
        // load tool
        tool.attachTo(lbr.getFlange());
        getLogger().info("Tool data loaded");

        // launch tcp server
        final Server tcpServer;
        try {
            tcpServer = new Server(InetAddress.getLocalHost().getHostAddress(),
                    30000);
        } catch (UnknownHostException e) {
            getLogger().info(e.getLocalizedMessage());
            return;
        }
        Thread serverThread = new Thread(tcpServer);
        serverThread.start();

        // wait until server is ready
        boolean isServerReady = false;
        while (!isServerReady) {
            if (tcpServer.getChannel() != null) {
                if (tcpServer.getChannel().isActive()) {
                    getLogger().info("Server is ready");
                    getLogger().info(
                            "Server channel bound to:\t"
                                    + tcpServer.getChannel().localAddress());
                    isServerReady = true;
                }
            }
        }

        // start test, sending scheduled info packets
        getLogger().info("Waiting for clients");
        boolean isClientConnected = false;
        while (!isClientConnected) {
            if (tcpServer.getChannelGroup().size() > 0) {
                isClientConnected = true;
                getLogger().info(
                        "Client connected:\t" + tcpServer.getChannelGroup());
            }
        }

        ScheduledExecutorService scheduler = Executors
                .newScheduledThreadPool(1);
        final ScheduledFuture<?> scheduledFuture = scheduler
                .scheduleAtFixedRate(new Runnable() {
                    @Override
                    public void run() {
                        RobotPacketGenerator robotPacketGenerator = new RobotPacketGenerator(lbr);
                        tcpServer.publish(robotPacketGenerator.generateStatusPacket(packetId++));
                    }
                }, 1000, 5, TimeUnit.MILLISECONDS);

        // move to forward starting pose
        getLogger().info("Moving to forward start position");
        lbr.move(LBRUtilities.ptpToHandGuidanceStart
                .setJointVelocityRel(LBRUtilities.SAFE_REL_JOINT_VELOCITY));

        // get start and end frame
        Frame startFrame = lbr.getCurrentCartesianPosition(tool
                .getDefaultMotionFrame());
        Frame endFrame = startFrame.copy();
        endFrame.setZ(-25);

        ForceCondition forceDetected = ForceCondition
                .createSpatialForceCondition(tool.getDefaultMotionFrame(),
                        maxForce);

        // create motions
        LIN upMotion = lin(startFrame).setCartVelocity(cartVel);
        LIN downMotion = lin(endFrame).setCartVelocity(cartVel).breakWhen(
                forceDetected);

        // execute motion loop
        for (int i = 0; i < 10; i++) {
            getLogger().info(String.format("Iteration:\t%d", i));
            IMotionContainer downMotionContrainer = tool.move(downMotion);
            if (downMotionContrainer.getFiredBreakConditionInfo() != null) {
                getLogger().info("Condition fired");
            }
            tool.move(upMotion);
        }

        // clean up
        getLogger().info("Closing");
        scheduledFuture.cancel(true);
        tcpServer.close();
        try {
            serverThread.join(5000);
        } catch (InterruptedException e) {
            getLogger().error(e.getLocalizedMessage());
        }

        getLogger().info("App finished");
    }
}
