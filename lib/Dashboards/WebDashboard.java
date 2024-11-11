package team696.frc.lib.Dashboards;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team696.frc.lib.Logging.PLog;

public class WebDashboard extends WebSocketServer {

    private static WebDashboard _dashboard;

    private final static int UPDATERATE = 20; // update in ms
    private final static int PORT = 5805;
    private static Verbosity VERBOSITY = Verbosity.minimal;

    private static HashMap<String, Supplier<Object>> _outgoingValues = new HashMap<String, Supplier<Object>>();
    private static HashMap<String, KeyInfo> _keyboardInputs = new HashMap<String, KeyInfo>();
    private static HashMap<String, IncomingInfo> _incomingValues = new HashMap<String, IncomingInfo>();

    /** Used to add values to send to dashboard. */
    public static void push(String name, Supplier<Object> value) { 
        _outgoingValues.put(name, value);
    }

    /** Register Functions to run based off of incoming values */
    public static IncomingInfo getData(String name) {
        return _incomingValues.getOrDefault(name, new IncomingInfo());
    }

    public static double timeSinceLastUpdate(String name) {
        if (!_incomingValues.containsKey(name)) return -1;
        return Timer.getFPGATimestamp() - _incomingValues.get(name).timestamp;
    }

    public static boolean getKeyValue(String name, boolean defaultValue) {
        if (!_keyboardInputs.containsKey(name) || 
            Timer.getFPGATimestamp() - _keyboardInputs.get(name).timestamp > 0.05) {
            return defaultValue;
        }
        return _keyboardInputs.get(name).pressed;
    }

    public static boolean getKeyValue(String name) {
        return getKeyValue(name, false);
    }

    // ex: WebDashboard.getKey("w").whileTrue(new InstantCommand(()->PLog.info("test", "hi")));
    public static Trigger getKey(String name) {
        return new BooleanEvent(CommandScheduler.getInstance().getDefaultButtonLoop(), (BooleanSupplier)(()->{return getKeyValue(name);}) ).castTo(Trigger::new);
    }

    private WebDashboard(int port) throws UnknownHostException   {
        super(new InetSocketAddress(port));
    }

    private WebDashboard(InetSocketAddress address) {
        super(address);
    }

    public static void Start() {
        try {
            _dashboard = new WebDashboard(PORT);
            _dashboard.setReuseAddr(true);
            _dashboard.start();  
        } catch (Exception e) {
            PLog.fatalException("WebDashboard", "Failed To Start Server.", e);   
        }
    }

    public static void Start(Verbosity verbosity) {
        VERBOSITY = verbosity;
        Start();
    }

    private static void dataReceived(String key, String value) {
        switch (key) {
            /* To Send Key Press information -> structure as "input:A,0,124124124", 
                which is a string containing input: and 0 or 1 for pressed or not and time when pressed on client */
            case "input": { 
                String[] parts = value.split(",");

                if (parts.length != 2) {
                    PLog.unusual("WebDashboard", "Malformed Key Information.");
                    return;
                }

                KeyInfo info = new KeyInfo(Integer.parseInt(parts[1]) == 1 ? true : false, Timer.getFPGATimestamp()); 
                _keyboardInputs.put(parts[0], info);

                break;
            }
            default: {           
                    _incomingValues.putIfAbsent(key, new IncomingInfo());
                    _incomingValues.get(key).update(value);
                break;
            }
        }
    }

    @SuppressWarnings("unchecked")
    private String dataToUpdate() {
        JSONObject obj = new JSONObject();

        obj.put("voltage", RobotController.getBatteryVoltage());
        obj.put("time", Timer.getMatchTime());
        

        JSONObject pose = new JSONObject();
        //pose.put("X", Swerve.get().getPose().getX());
        //pose.put("Y", Swerve.get().getPose().getY());
        //pose.put("R", Swerve.get().getPose().getRotation().getDegrees());
        obj.put("pose", pose);

        obj.put("debug", new JSONArray());

        for (String key : _outgoingValues.keySet()) {
            obj.put(key, _outgoingValues.get(key).get());
        }

        return obj.toJSONString();
    }  

    @SuppressWarnings("unchecked")
    private String dataToInitialize() {
    
        JSONObject obj = new JSONObject();

        obj.put("selected_auto", "test");
        obj.put("auto", new String[] {"auto 1", "auto 2"});

        return obj.toJSONString();
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        PLog.info("WebDashboard", conn.getRemoteSocketAddress().getHostName() + " has opened a connection.");

        (new Thread(() -> {
            conn.send(dataToInitialize());

            while (conn.isOpen()) {

                conn.send(dataToUpdate());

                sleep(UPDATERATE);
            }
        })).start();
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        _keyboardInputs.clear();
        PLog.info("WebDashboard", conn.getRemoteSocketAddress().getHostName() + " has closed its connection.");
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        String[] parts = message.split(":"); // Message Comes in "Key:Value pair"
        dataReceived(parts[0], parts[1]);

        if (VERBOSITY.level >= Verbosity.verbose.level)
            PLog.info(conn.getRemoteSocketAddress().getHostName() + "->" + _dashboard.getAddress().getHostName(), message);
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        _keyboardInputs.clear();
        PLog.fatalException("WebDashboard", "Error", ex);
    }

    @Override
    public void onStart() {
        PLog.info("WebDashboard", String.format("WebDashboard has started on %s:%d.", _dashboard.getAddress().getHostName(), PORT));
    }

    public static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
            PLog.fatalException("Sleep", "Failed To Sleep", e);
        }
    }

    public enum Verbosity {
        minimal(0),
        verbose(1);

        public final int level;

        private Verbosity(int l) {
            level = l;
        }
    }

    private static class KeyInfo {
        public boolean pressed;
        public double timestamp;

        public KeyInfo(boolean p, double t) {
            pressed = p;
            timestamp = t;
        }
    }

    public static class IncomingInfo {
        public String latestValue;
        public double timestamp;

        public IncomingInfo() {
            this.latestValue = "";
            this.timestamp = 0;
        }

        public void update(String value) {
            this.latestValue = value;
            this.timestamp = Timer.getFPGATimestamp();
        }

        public String get() {
            if (latestValue == "" || Timer.getFPGATimestamp() - timestamp > 0.05) return "";

            return latestValue;
        }

        public Double getDouble() {
            if (latestValue == "" || Timer.getFPGATimestamp() - timestamp > 0.05) return 0.0;

            return Double.parseDouble(latestValue);
        }
    }
}