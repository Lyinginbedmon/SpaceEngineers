// == Hermit Crab Delivery Drone OS ==
const string version = "2.5";
const String spinning = "-\\|/";
const String config_default = 
    "[setup]\n;Name of the forward indicator light\nindicator_light=Interior Light\n\n[travel]\n;Style of travel (either forward or lateral)\nmove_style=forward\n;Top speed for this drone\nmax_speed=50\n;Yaw offset for this drone to face a given direction\nyaw_offset=0\n\n[altitude]\n;Whether this drone should ignore cockpit altitude readings\nignore_altitude=false\n;Minimum altitude this drone should maintain during travel\nmin_altitude=50";

// Drone model
public static Drone model;
// Indicator light
public static IMyLightingBlock light;
public static IMyTextPanel screen;

// Config variables
private static MyIni config = new MyIni();
private static string lightName;
public static MoveStyle moveStyle;
public static double maxSpeed;
public static float yawOffset;
public static bool ignoreAltitude;
public static int minAltitude;

readonly static Matrix directionMatrix = new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
private static Dictionary<string, State> STATE_MAP = new Dictionary<String, State>();
private string state = "idle";

public static int targetGPS = -1;
private static Vector3D destination;

private static int ticksRunning;
private bool hasClearedScreen;

public enum MoveStyle
{
    FORWARD,
    LATERAL
}
private static MoveStyle styleFromName(String nameIn)
{
    nameIn = nameIn.ToLower();
    foreach(MoveStyle style in Enum.GetValues(typeof(MoveStyle)))
        if(Enum.GetName(typeof(MoveStyle), style).ToLower() == nameIn)
            return style;
    return MoveStyle.FORWARD;
}

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    registerState("stop", new StateStop());
    registerState("stabilise", new StateStabilise());
    registerState("rise", new StateRise());
    registerState("point", new StatePoint());
    registerState("move", new StateMove());
    registerState("lateral", new StateMoveLateral());
    registerState("land", new StateLand());
    registerState("land2", new StateLandPrecise());
    
    Me.CustomName = "Drone Control";
    if(Me.CustomData.Length == 0)
        Me.CustomData = config_default;
    loadData(Me.CustomData);
    
    model = new Drone(Me, this);
    light = (IMyLightingBlock)GridTerminalSystem.GetBlockWithName(lightName);
    if(light != null)
    {
        light.Enabled = true;
        light.Color = new Color(255, 255, 255);
        light.BlinkIntervalSeconds = 0F;
    }
    
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid)
        {
            screen = panel;
            break;
        }
}

public void Save()
{
    Storage = String.Join(";", state, targetGPS);
}

public void Main(string argument, UpdateType updateSource)
{
    hasClearedScreen = false;
    ++ticksRunning;
    Echo("Hermit Crab OS Version "+version+" "+getSpinning());
    
    if(argument.Length > 0)
        parseArg(argument);
    loadStorage(Storage);
    
    model.tick(Echo);
    if(!model.diagnostic(Echo))
        setState("idle");
    
    MyWaypointInfo waypoint = getWaypoint();
    if(waypoint.IsEmpty())
        return;
    destination = waypoint.Coords;
    Echo("\nDestination: "+waypoint.Name);
    Echo("Coords: "+String.Join(", ", Math.Round(destination.X,2), Math.Round(destination.Y,2), Math.Round(destination.Z,2)));
    
    Echo("Available States:");
    foreach(var key in STATE_MAP.Keys)
        Echo((key==state ? " > " : " * ")+key);
    Echo("idle"==state ? " > idle" : " * idle");
    if(state != "idle")
    {
        echoToScreens("State: "+state);
        echoToScreens("Status Report:");
    }
    if(STATE_MAP.ContainsKey(state))
    {
        State currentState = STATE_MAP[state];
        currentState.tick(destination, model, echoToScreens);
        if(currentState.invalid(model))
            setState(currentState.exit(model));
    }
    else
        setState("idle");
    
    Save();
}

public void loadData(String customData)
{
    MyIniParseResult result;
    if(!config.TryParse(customData, out result))
    {
        Runtime.UpdateFrequency = UpdateFrequency.Once;
        throw new Exception(result.ToString());
    }
    
    lightName = config.Get("setup", "indicator_light").ToString("Interior Light");
    
    moveStyle = styleFromName(config.Get("travel", "move_style").ToString("forward"));
    yawOffset = (float)config.Get("travel", "yaw_offset").ToDouble(0D);
    maxSpeed = config.Get("travel", "max_speed").ToDouble(50D);
    
    ignoreAltitude = config.Get("altitude", "ignore_altitude").ToBoolean(false);
    minAltitude = config.Get("altitude", "minimum_altitude").ToInt32(50);
}

public void parseArg(String argument)
{
    String val = argument.ToLower();
    if(val == "reset")
        reset();
    else if(val == "remodel")
        model = new Drone(Me, this);
    else if(val.StartsWith("set_gps="))
        targetGPS = (int)clamp(int.Parse(val.Split('=')[1]), -1, 1);
    else if(val.StartsWith("set_state="))
        setState(val.Split('=')[1]);
    else if(val == "deliver" && model.getTotalChargePercent() > 0.8F)
    {
        targetGPS = (int)clamp(targetGPS, 0, 1) == 0 ? 1 : 0;
        setState("rise");
    }
    
    Save();
}

public void loadStorage(String memory)
{
    reset();
    String[] lines = memory.ToLower().Split('|');
    int entries = lines.Length;
    
    // Operating data
    String[] data = lines[0].Split(';');
    if(data.Length == 2)
    {
        setState(data[0]);
        targetGPS = int.Parse(data[1]);
    }
    if(--entries <= 0) return;
}

public void echoToScreens(string text)
{
    Echo(text);
    if(screen != null && screen.IsFunctional && screen.Enabled)
        screen.WriteText((hasClearedScreen ? "\n" : "") + text, hasClearedScreen);
    hasClearedScreen = true;
}

private void reset()
{
    Save();
}

private static void registerState(String nameIn, State stateIn)
{
    if(nameIn == "" || stateIn == null)
        return;
    
    nameIn = nameIn.ToLower();
    STATE_MAP[nameIn] = stateIn;
}

// Exits the current state and moves to the first recognised one
private void setState(String statesIn)
{
    statesIn = statesIn.ToLower();
    string enterState = STATE_MAP.ContainsKey(statesIn) ? statesIn : "idle";
    
    if(enterState != state)
    {
        if(STATE_MAP.ContainsKey(state))
            STATE_MAP[state].exit(model);
        
        state = "";
        Save();
        
        state = enterState;
        if(STATE_MAP.ContainsKey(state))
        {
            State enteredState = STATE_MAP[state];
            enteredState.enter(destination, model);
            light.Color = enteredState.indicator;
        }
    }
    
    Save();
}

public MyWaypointInfo getWaypoint()
{
    return model.getWaypoint(targetGPS);
}

// #### UTILITY FUNCTIONS ####
public static char getSpinning()
{
    return spinning[ticksRunning % spinning.Length];
}

public static Vector3 getLocalFacing(IMyCubeBlock block, IMyRemoteControl remote)
{
    Matrix fromGridToReference;
    remote.Orientation.GetMatrix(out fromGridToReference);
    Matrix.Transpose(ref fromGridToReference, out fromGridToReference);
    
    Matrix fromThrusterToGrid;
    block.Orientation.GetMatrix(out fromThrusterToGrid);
    return Vector3.Transform(fromThrusterToGrid.Backward, fromGridToReference);
}

public static float clamp(float val, float min, float max)
{
    float minVal = Math.Min(min, max);
    float maxVal = Math.Max(min, max);
    return Math.Max(minVal, Math.Min(maxVal, val));
}

public static bool lowAltitudeWarning(int offset)
{
    return model.collisionAlarm() || !ignoreAltitude && model.getAltitude() < (minAltitude - offset);
}

public class Drone
{
    // Remote control
    private IMyRemoteControl remote;
    // Gyroscopes
    private List<IMyGyro> gyros = new List<IMyGyro>();
    // Terrain sensor
    private IMySensorBlock sensor;
    // Docking connector
    private IMyShipConnector connector;
    // All batteries
    private List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
    // All thrusters
    private List<IMyThrust> thrusters = new List<IMyThrust>();
    private Dictionary<Base6Directions.Direction, List<IMyThrust>> thrustMap = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
    private PID pidX = new PID(), pidY = new PID(), pidZ = new PID();
    private Vector3D targetVelocity = new Vector3D(0, 0, 0);
    private Vector3D targetFacing = new Vector3D(0, 0, 0);
    
    private Vector3D forwRef;
    private Vector3D downRef;
    private Vector3D gravTarget;
    
    public Drone(IMyProgrammableBlock me, MyGridProgram program)
    {
        List<IMyRemoteControl> remotes = new List<IMyRemoteControl>();
        program.GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(remotes);
        foreach(var controller in remotes)
            if(controller.CubeGrid == me.CubeGrid && controller.IsFunctional)
            {
                controller.CustomName = "Drone Remote";
                controller.DampenersOverride = true;
                remote = controller;
                break;
            }
        
        List<IMyGyro> gyros2 = new List<IMyGyro>();
        program.GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyros2);
        foreach(var gyro in gyros2)
            if(gyro.CubeGrid == me.CubeGrid)
                gyros.Add(gyro);
        
        foreach(var direction in Base6Directions.EnumDirections)
            thrustMap[direction] = new List<IMyThrust>();
        if(remote != null)
        {
            List<IMyThrust> thrusters2 = new List<IMyThrust>();
            program.GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters2);
            foreach(var thruster in thrusters2)
                if(thruster.CubeGrid == me.CubeGrid)
                {
                    thruster.ShowInTerminal = false;
                    thruster.ShowInToolbarConfig = false;
                    
                    Vector3 thrustVec = getLocalFacing(thruster, remote);
                    Base6Directions.Direction localFace = Base6Directions.Direction.Forward;
                    if(thrustVec == directionMatrix.Forward)
                        localFace = Base6Directions.Direction.Forward;
                    else if(thrustVec == directionMatrix.Backward)
                        localFace = Base6Directions.Direction.Backward;
                    else if(thrustVec == directionMatrix.Left)
                        localFace = Base6Directions.Direction.Left;
                    else if(thrustVec == directionMatrix.Right)
                        localFace = Base6Directions.Direction.Right;
                    else if(thrustVec == directionMatrix.Up)
                        localFace = Base6Directions.Direction.Up;
                    else if(thrustVec == directionMatrix.Down)
                        localFace = Base6Directions.Direction.Down;
                    else
                    {
                        thruster.Enabled = false;
                        thruster.CustomName = "Unrecognised Thruster";
                        continue;
                    }
                    
                    thrusters.Add(thruster);
                    registerThruster(thruster, localFace);
                    thruster.CustomName = "Thruster "+(Enum.GetName(typeof(Base6Directions.Direction), localFace)[0]+""+getThrusters(localFace).Count);
                }
            
            List<IMyShipConnector> connectors = new List<IMyShipConnector>();
            program.GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
            foreach(var connect in connectors)
                if(connect.CubeGrid == me.CubeGrid && getLocalFacing(connect, remote) == directionMatrix.Up)
                {
                    connect.CustomName = "Docking Connector";
                    connect.Enabled = true;
                    connect.ShowInToolbarConfig = false;
                    
                    connector = connect;
                }
        }
        
        List<IMySensorBlock> sensors = new List<IMySensorBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMySensorBlock>(sensors);
        foreach(var sense in sensors)
            if(sense.CubeGrid == me.CubeGrid && sense.IsFunctional)
            {
                sense.CustomName = "Terrain Sensor";
                sense.Enabled = true;
                
                Vector3 faceVec = getLocalFacing(sense, remote);
                sense.BackExtend = faceVec.Y <= 0 ? int.MaxValue : 0;
                sense.FrontExtend = faceVec.Y >= 0 ? int.MaxValue : 0;
                sense.RightExtend = sense.LeftExtend = sense.TopExtend = sense.BottomExtend = int.MaxValue;
                
                sense.DetectAsteroids = true;
                sense.DetectFloatingObjects = false;
                sense.DetectLargeShips = true;
                sense.DetectStations = true;
                sense.DetectPlayers = false;
                sense.DetectSmallShips = false;
                sense.DetectSubgrids = false;
                
                sense.ShowInToolbarConfig = false;
                sensor = sense;
                break;
            }
        
        List<IMyBatteryBlock> bats = new List<IMyBatteryBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMyBatteryBlock>(bats);
        foreach(var battery in bats)
            if(battery.CubeGrid == me.CubeGrid)
            {
                battery.CustomName = "Battery "+batteries.Count;
                battery.ShowInTerminal = false;
                battery.ShowInToolbarConfig = false;
                batteries.Add(battery);
            }
    }
    
    public void tick(Action<string> echo)
    {
        if(remote == null || !remote.IsFunctional || remote.IsUnderControl)
        {
            resetGyros();
            resetThrusters(thrusters);
            return;
        }
        
        updateOrientationRef();
        updateLevelling();
        
        // Total mass of ship (kg)
        double mass = remote.CalculateShipMass().TotalMass;
        
        // Downward gravity relative to ship (m/s)
        double grav = toLocal(getGravity()).Length();
        
        // Velocity relative to ship (m/s)
        Vector3D velocity = toLocal(remote.GetShipVelocities().LinearVelocity);
        
        // Target velocity relative to ship (m/s)
        Vector3D targetVel = toLocal(targetVelocity);
        
        handleThrust(targetVel.X, velocity.X, pidX, mass, getThrusters(Base6Directions.Direction.Right), getThrusters(Base6Directions.Direction.Left));
        handleThrust(targetVel.Y, velocity.Y, pidY, mass, getThrusters(Base6Directions.Direction.Up), Math.Abs(grav)*mass);
        handleThrust(targetVel.Z, velocity.Z, pidZ, mass, getThrusters(Base6Directions.Direction.Backward), getThrusters(Base6Directions.Direction.Forward));
    }
    
    // Runs a diagnostic of all vital components and returns true if they all pass
    public bool diagnostic(Action<string> echo)
    {
        bool pass = true;
        if(remote == null || !remote.IsFunctional)
        {
            echo(" X Missing remote control");
            pass = false;
        }
        else
        {
            List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
            remote.GetWaypointInfo(waypoints);
            
            if(waypoints.Count < 2)
            {
                echo(" X Not enough waypoints registered");
                pass = false;
            }
        }
        
        foreach(var direction in Base6Directions.EnumDirections)
        {
            List<IMyThrust> set = getThrusters(direction);
            string group = Enum.GetName(typeof(Base6Directions.Direction), direction);
            if(set.Count == 0)
            {
                echo(" X Missing thrusters for "+group);
                pass = false;
            }
            else
            {
                bool func = false;
                foreach(var thruster in set)
                    if(thruster != null && thruster.IsFunctional)
                    {
                        func = true;
                        break;
                    }
                if(!func)
                {
                    echo(" X No functional thrusters for "+group);
                    pass = false;
                }
            }
        }
        
        if(gyros.Count == 0)
        {
            echo(" X Missing gyroscopes");
            pass = false;
        }
        else
        {
            bool func = false;
            foreach(var gyro in gyros)
                if(gyro != null && gyro.IsFunctional)
                {
                    func = true;
                    break;
                }
            if(!func)
            {
                echo(" X No functional gyroscopes");
                pass = false;
            }
        }
        
        if(sensor == null || !sensor.IsFunctional)
        {
            echo(" X No terrain collision sensor");
            pass = false;
        }
        
        if(connector == null || !connector.IsFunctional)
        {
            echo(" X No functional docking connector");
            pass = false;
        }
        
        if(light == null)
            echo(" ? Missing indicator light named "+lightName);
        
        return pass;
    }

    public MyWaypointInfo getWaypoint(int index)
    {
        List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
        remote.GetWaypointInfo(waypoints);
        index = (int)clamp(index, 0, waypoints.Count - 1);
        
        return waypoints[index];
    }
    
    public IMyRemoteControl getRemote(){ return remote; }
    public Vector3D getGravity(){ return remote == null ? new Vector3D(0, 0, 0) : remote.GetNaturalGravity(); }
    public Vector3D getWorldPosition(){ return connector.GetPosition(); }
    
    public Vector3D toLocal(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, MatrixD.Transpose(remote.WorldMatrix)); }
    public Vector3D toWorld(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, remote.WorldMatrix); }
    
    public void setSensor(bool onOff){ if(sensor == null) return; sensor.Enabled = onOff; }
    
    public double getAltitude()
    {
        double altSea = 0D;
        double altVox = 0D;
        remote.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out altSea);
        remote.TryGetPlanetElevation(MyPlanetElevation.Surface, out altVox);
        return Math.Min(altSea, altVox);
    }

    public bool collisionAlarm(){ return sensor != null && sensor.Enabled && sensor.IsActive; }
    
    public IMyShipConnector getConnector(){ return connector; }
    public void setConnector(bool on){ connector.Enabled = on; }
    public bool isConnected(){ return connector.Status == MyShipConnectorStatus.Connected; }
    public MyShipConnectorStatus getConnectorStatus(){ return connector.Status; }
    public void toggleConnect(){ connector.ToggleConnect(); }
    
    public void setTargetFacing(Vector3D face){ targetFacing = face; }
    public void clearTargetFacing(){ setTargetFacing(new Vector3D(0, 0, 0)); }
    
    // Sets gyroscopes to maintain gravity level on Pitch and Roll axises
    private void updateLevelling()
    {
        resetGyros();
        double error = currentFacingError();
        double currentSpin = toLocal(getShipRotations()).Y;
        double spin = targetFacing.Length() > 0 && Math.Abs(error) > 1 && currentSpin < 5 ? -MathHelper.Clamp(error, -10, 10) * 0.1D : 0D;
        
        foreach(var gyro in gyros)
        {
            if(gyro == null || !gyro.Enabled || !gyro.IsFunctional) continue;
            gyro.GyroOverride = true;
            Matrix localOrientation;
            gyro.Orientation.GetMatrix(out localOrientation);
            
            if(Vector3D.TransformNormal(Vector3D.Up, gyro.WorldMatrix) == toWorld(Vector3D.Up))
                gyro.Yaw = (float)spin;
            
            var axisPR = getGyroVec(gravTarget, downRef, gyro.WorldMatrix.GetOrientation(), localOrientation);
            gyro.Pitch = (float)-axisPR.X;
            gyro.Roll = (float)-axisPR.Z;
        }
    }

    private Vector3D getGyroVec(Vector3D targetVec, Vector3D refVec, MatrixD orientation, Matrix localOrientation)
    {
        var localReference = Vector3D.Transform(refVec, MatrixD.Transpose(localOrientation));
        var localTarget = Vector3D.Transform(targetVec, MatrixD.Transpose(orientation));
        
        var axis = Vector3D.Cross(localReference, localTarget);
        var angle = axis.Length();
        angle = Math.Atan2(angle, Math.Sqrt(Math.Max(0, 1 - angle * angle)));
        if(Vector3D.Dot(localReference, localTarget) < 0)
            angle = Math.PI;
        
        return axis;
    }
    
    public float currentFacingError()
    {
        return getFacingError(targetFacing);
    }
    
    public float getFacingError(Vector3D vector)
    {
        Vector3D forwLocal = toLocal(vector);
        double yawV = Math.Atan2(forwLocal.Z, forwLocal.X);
        double yawL = Math.Atan2(forwRef.Z, forwRef.X);
        return (float)MathHelper.ToDegrees(yawL - yawV) + yawOffset;
    }

    // Updates the orientation values used for navigation and levelling
    private void updateOrientationRef()
    {
        Matrix shipOrientation;
        remote.Orientation.GetMatrix(out shipOrientation);
        
        var quatPitch = Quaternion.CreateFromAxisAngle(shipOrientation.Left, 0F);
        var quatRoll = Quaternion.CreateFromAxisAngle(shipOrientation.Backward, 0F);
        
        forwRef = Vector3D.Transform(shipOrientation.Forward, quatPitch * quatRoll);
        downRef = Vector3D.Transform(shipOrientation.Down, quatPitch * quatRoll);
        gravTarget = remote.GetNaturalGravity();
    }

    // Resets gyroscopes to prevent further rotation after state change
    private void resetGyros()
    {
        foreach(var gyro in gyros)
        {
            gyro.GyroOverride = false;
            gyro.Pitch = 0F;
            gyro.Yaw = 0F;
            gyro.Roll = 0F;
        }
    }

    public float getAvailableChargePercent()
    {
        float charge = 0F;
        float maxCharge = 0F;
        foreach(var battery in batteries)
            if(battery != null && battery.IsFunctional && battery.Enabled && battery.ChargeMode != ChargeMode.Recharge)
            {
                charge += battery.CurrentStoredPower;
                maxCharge += battery.MaxStoredPower;
            }
        
        return charge / maxCharge;
    }
    
    public float getTotalChargePercent()
    {
        float charge = 0F;
        float maxCharge = 0F;
        foreach(var battery in batteries)
            if(battery != null && battery.IsFunctional)
            {
                charge += battery.CurrentStoredPower;
                maxCharge += battery.MaxStoredPower;
            }
        
        return charge / maxCharge;
    }
    
    public void doCharging()
    {
        IMyBatteryBlock lowest = null;
        float lowestCharge = float.MaxValue;
        foreach(var battery in batteries)
        {
            battery.ChargeMode = ChargeMode.Auto;
            float charge = battery.CurrentStoredPower / battery.MaxStoredPower;
            if(charge < lowestCharge)
            {
                lowest = battery;
                lowestCharge = charge;
            }
        }
        if(batteries.Count > 1)
            lowest.ChargeMode = ChargeMode.Recharge;
    }
    
    public void resetBatteries()
    {
        foreach(var battery in batteries)
            if(battery != null)
            {
                battery.Enabled = true;
                battery.ChargeMode = ChargeMode.Auto;
            }
    }
    
    public Vector3D getShipVelocities(){ return remote.GetShipVelocities().LinearVelocity; }
    public Vector3D getShipRotations(){ return remote.GetShipVelocities().AngularVelocity; }
    
    private void registerThruster(IMyThrust thruster, Base6Directions.Direction direction)
    {
        List<IMyThrust> set = thrustMap[direction];
        set.Add(thruster);
        thrustMap[direction] = set;
    }
    
    public void setTargetVelocity(Vector3D vel){ targetVelocity = vel; }
    public void clearTargetVelocity(){ targetVelocity = new Vector3D(0, 0, 0); }
    
    private List<IMyThrust> getThrusters(Base6Directions.Direction face)
    {
        return thrustMap[face];
    }
    
    private void resetThrusters(List<IMyThrust> group)
    {
        foreach(var thruster in group)
        {
            if(thruster == null) continue;
            thruster.Enabled = true;
            thruster.ThrustOverride = 0F;
        }
    }
    
    private void handleThrust(double target, double current, PID pid, double mass, List<IMyThrust> group, double offset)
    {
        handleThrust(target, current, pid, mass, group, new List<IMyThrust>(), offset);
    }
    
    private void handleThrust(double target, double current, PID pid, double mass, List<IMyThrust> groupA, List<IMyThrust> groupB)
    {
        handleThrust(target, current, pid, mass, groupA, groupB, 0D);
    }
    
    private void handleThrust(double target, double current, PID pid, double mass, List<IMyThrust> groupA, List<IMyThrust> groupB, double offset)
    {
        int tallyA = 0;
        foreach(var thruster in groupA) if(thruster != null && thruster.IsFunctional) tallyA++;
        
        int tallyB = 0;
        foreach(var thruster in groupB) if(thruster != null && thruster.IsFunctional) tallyB++;
        
        if(target != 0F)
        {
            double pidVal = pid.calculate(target, current) * mass;
            
            // Total thrust needed (N)
            double totalThrust = offset + pidVal;
            
            foreach(var thruster in groupA)
            {
                if(thruster == null || !thruster.IsFunctional) continue;
                thruster.Enabled = totalThrust > 0F;
                thruster.ThrustOverride = thruster.Enabled ? (float)totalThrust / tallyA : 0F;
            }
            
            totalThrust -= offset;
            foreach(var thruster in groupB)
            {
                if(thruster == null || !thruster.IsFunctional) continue;
                thruster.Enabled = totalThrust < 0F;
                thruster.ThrustOverride = thruster.Enabled ? (float)Math.Abs(totalThrust) / tallyB : 0F;
            }
        }
        else
        {
            // Just use regular dampening if we just want to not move
            resetThrusters(groupA);
            resetThrusters(groupB);
            pid.reset();
        }
    }
    
    public class PID
    {
        // Interval time
        private double DT = 1D;
        // Proportional gain
        private double KP = 1D;
        //  Integral gain
        private double KI = 1D;
        // Derivative gain
        private double KD = 1D;
        
        private double integral = 0D;
        private double prevError = 0D;
        
        public PID(double rateIn, double gainIn) : this(rateIn, gainIn, gainIn, gainIn){ }
        public PID() : this(1D,2.5D){ }
        public PID(double rate, double propGain, double intGain, double derGain)
        {
            DT = rate;
            KP = propGain;
            KI = intGain;
            KD = derGain;
        }
        
        public double calculate(double setPoint, double pv)
        {
            double error = setPoint - pv;
            
            double propOut = KP * error;
            
            integral += error * DT;
            double intOut = KI * integral;
            
            double derivative = (error - prevError) / DT;
            double derOut = KD * derivative;
            
            double output = propOut + intOut + derOut;
            
            prevError = error;
            return output;
        }
        
        public void reset()
        {
            prevError = 0D;
            integral = 0D;
        }
    }
}

public abstract class State
{
    private Controls[] affectedComponents = new Controls[0];
    public Color indicator = new Color(255, 255, 255);
    
    protected State(Controls[] affectedIn) : this(new Color(255, 255, 255), affectedIn){ }
    protected State(Color colorIn, Controls[] affectedIn)
    {
        indicator = colorIn;
        affectedComponents = affectedIn;
    }
    
    // Performs any basic initial settings this state needs
    public virtual void enter(Vector3D destination, Drone model){ }
    // Performs the functions of this state
    public abstract void tick(Vector3D destination, Drone model, Action<string> echo);
    // Returns true if a state change should occur, ie. this state's work is finished and it can safely stop
    public abstract bool invalid(Drone model);
    // Resets any changes caused by this state and returns the state names that should be moved into
    public virtual String exit(Drone model)
    {
        resetModel(model);
        onExit(model);
        return exitState(model);
    }
    public virtual void onExit(Drone model){ }
    public virtual String exitState(Drone model){ return "idle"; }
    
    protected virtual void resetModel(Drone model)
    {
        foreach(var component in affectedComponents)
            switch(component)
            {
                case Controls.THRUSTERS: model.clearTargetVelocity(); break;
                case Controls.GYROS: model.clearTargetFacing(); break;
                case Controls.CONNECTOR: model.setConnector(true); if(model.isConnected()) model.toggleConnect();; break;
                case Controls.BATTERIES: model.resetBatteries(); break;
                case Controls.SENSOR: model.setSensor(true); break;
                case Controls.LIGHT: light.Enabled = true; break;
            }
    }
    
    protected enum Controls
    {
        THRUSTERS,
        GYROS,
        CONNECTOR,
        BATTERIES,
        SENSOR,
        LIGHT
    }
}

public class StateStop : State
{
    public StateStop() : base(new Controls[]{Controls.BATTERIES, Controls.SENSOR, Controls.LIGHT, Controls.CONNECTOR}){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        float charge = model.getTotalChargePercent();
        echo("   > Charge: "+Math.Round(charge, 2)*100+"%");
        echo("   > "+(charge >= 0.8F ? "Launch ready" : "Recharging"));
        model.doCharging();
        model.setSensor(false);
    }
    
    public override bool invalid(Drone model){ return model.getAltitude() > 5 && !model.isConnected(); }
    public override String exitState(Drone model){ return "land"; }
}

public class StateStabilise : State
{
    public StateStabilise() : base(new Color(255, 0, 255), new Controls[0]){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo){ }
    public override bool invalid(Drone model){ return model.getShipRotations().Length() < 0.5D; }
    public override String exitState(Drone model){ return "point"; }
}

public class StateRise : State
{
    public StateRise() : base(new Color(255, 0, 0), new Controls[]{Controls.THRUSTERS}){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        double altitude = model.getAltitude();
        echo("   > Altitude: "+Math.Round(altitude,2));
        
        if(!lowAltitudeWarning(0))
            model.clearTargetVelocity();
        else
        {
            double speed = model.collisionAlarm() ? 15 : clamp(minAltitude - (int)altitude, 1, 100);
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            
            model.setTargetVelocity(Vector3D.Multiply(gravity, -speed));
        }
    }
    
    public override bool invalid(Drone model){ return !lowAltitudeWarning(0); }
    public override String exitState(Drone model){ return "point"; }
}

public class StatePoint : State
{
    public StatePoint() : base(new Color(255, 165, 0), new Controls[]{Controls.GYROS}){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        if(moveStyle != MoveStyle.FORWARD)
            return;
        
        Vector3D direction = (destination - model.getWorldPosition());
        direction.Normalize();
        model.setTargetFacing(direction);
        
        echo(" > Error: "+Math.Round(model.currentFacingError(), 2));
    }
    
    public override bool invalid(Drone model)
    {
        return moveStyle != MoveStyle.FORWARD || Math.Abs(model.currentFacingError()) < 1;
    }
    public override String exitState(Drone model){ return "move"; }
}

public class StateMove : State
{
    protected Vector3D dest;
    private double cutoffDist;
    protected String exitCause = "";
    
    public StateMove() : this(20D, new Color(0, 255, 255)){ }
    public StateMove(Color colorIn) : this(20D, colorIn){ }
    protected StateMove(double distIn, Color colorIn) : base(colorIn, new Controls[]{Controls.THRUSTERS})
    {
        cutoffDist = distIn;
    }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        dest = destination;
        Vector3D direction = getLateralDirection(model);
        double speedLimit = getSpeedLimit(destination, model);
        
        Vector3D velocities = model.toLocal(model.getShipVelocities());
        echo("   > Distance: "+Math.Round(direction.Length(), 2));
        echo("   > Max Velocity Per Axis: "+Math.Round(speedLimit, 2)+"m/s");
        echo("   > Velocities: "+String.Join(", ", Math.Abs(Math.Round(velocities.X,2)), Math.Abs(Math.Round(velocities.Y,2)), Math.Abs(Math.Round(velocities.Z,2))));
        echo("     > "+Math.Round(velocities.Length(), 2)+"m/s");
        
        if(direction.Length() > cutoffDist)
        {
            direction.Normalize();
            model.setTargetVelocity(new Vector3D(speedLimit * direction.X, speedLimit * direction.Y, speedLimit * direction.Z));
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model)
    {
        if(lowAltitudeWarning(10))
        {
            exitCause = "rise";
            return true;
        }
        else if(moveStyle == MoveStyle.FORWARD && Math.Abs(model.getFacingError(destination - model.getWorldPosition())) > 5D)
        {
            exitCause = "point";
            return true;
        }
        else if(getLateralDirection(model).Length() <= cutoffDist)
        {
            exitCause = "lateral";
            return true;
        }
        
        return false;
    }
    
    public override String exitState(Drone model){ return exitCause; }
    
    protected virtual Vector3D getLateralDirection(Drone model)
    {
        Vector3D world = model.toLocal(dest - model.getWorldPosition());
        world.Y = 0;
        return model.toWorld(world);
    }
    
    protected virtual double getVerticalDistance(Drone model)
    {
        Vector3D world = dest - model.getWorldPosition();
        world = model.toLocal(world);
        return Math.Abs(world.Y);
    }
    
    protected virtual double getSpeedLimit(Vector3D destination, Drone model){ return Math.Min(maxSpeed, getLateralDirection(model).Length() / 10); }
}

public class StateMoveLateral : StateMove
{
    public StateMoveLateral() : base(0.5D, new Color(0, 255, 0)){ }
    
    protected override double getSpeedLimit(Vector3D destination, Drone model){ return Math.Max(getLateralDirection(model).Length() / 10, 0.05D); }
    
    public override bool invalid(Drone model)
    {
        double direction = getLateralDirection(model).Length();
        if(lowAltitudeWarning(10))
        {
            exitCause = "rise";
            return true;
        }
        else if(direction > 20)
        {
            exitCause = "move";
            return true;
        }
        else if(direction < 0.5D)
        {
            exitCause = "land";
            return true;
        }
        
        return false;
    }
}

public class StateLand : StateMove
{
    private bool prevCollision = false;
    
    public StateLand() : this(new Color(0, 255, 255)){ }
    protected StateLand(Color colorIn) : base(colorIn){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        dest = destination;
        Vector3D deviation = getLateralDirection(model);
        double distVert = (destination - model.getWorldPosition()).Length() - deviation.Length();
        echo("   > Altitude: "+Math.Round(distVert,2));
        echo("   > Deviation: "+Math.Round(deviation.Length(),2));
        
        if(distVert > 1 && !model.isConnected())
        {
            Vector3D direction = (dest - model.getWorldPosition());
            direction.Normalize();
            
            model.setTargetVelocity(Vector3D.Multiply(direction, model.collisionAlarm() ? Math.Min(2D, distVert) : 15D));
        }
        else
            model.clearTargetVelocity();
        
        if(model.getConnectorStatus() == MyShipConnectorStatus.Connectable)
            model.toggleConnect();
    }
    
    public override bool invalid(Drone model)
    {
        Vector3D deviation = getLateralDirection(model);
        if((!ignoreAltitude && model.getAltitude() < 5) || model.isConnected())
        {
            exitCause = "stop";
            return true;
        }
        else if(deviation.Length() > 1 && prevCollision != model.collisionAlarm())
        {
            exitCause = "land2";
            return true;
        }
        prevCollision = model.collisionAlarm();
        return false;
    }
}

public class StateLandPrecise : StateLand
{
    public StateLandPrecise() : base(new Color(0, 0, 255)){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        dest = destination;
        Vector3D direction = getLateralDirection(model);
        Vector3D velocities = model.toLocal(model.getShipVelocities());
        
        echo("   > Distance: "+Math.Round(direction.Length(),2));
        echo("   > Velocities: "+String.Join(", ", Math.Abs(Math.Round(velocities.X,2)), Math.Abs(Math.Round(velocities.Z,2))));
        
        if(direction.Length() > 0.5D)
        {
            direction.Normalize();
            model.setTargetVelocity(direction);
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model){ return getLateralDirection(model).Length() < 0.5D; }
    public override String exitState(Drone model){ return "land"; }
}