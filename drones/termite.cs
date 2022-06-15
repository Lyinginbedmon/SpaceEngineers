// == Termite Miner OS ==
const string version = "1.0";
const int GRID_SIZE = 10;
const double STEP_DIST = 10D;

private IMyTextPanel screen;
private bool hasClearedScreen = false;

private static GridStyle STYLE = GridStyle.CORNER_SPIRAL;
private static int rotation = 0;
private static int index=0;

private static bool hasHomePos = false;
private static Vector3D homePos = new Vector3D(0,0,0);
private static Vector3D moveForw = new Vector3D(0, 0, 1);
private static Vector3D moveRight = new Vector3D(-1, 0, 1);
readonly static Matrix directionMatrix = new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

static Dictionary<string, State> STATE_MAP = new Dictionary<String, State>();
private string state = "idle";
private int ticksInvalid = 0;

public Drone model;
public static int boreDepth = 40;
private static MyItemType[] ores = new MyItemType[]
    {
        MyItemType.MakeOre("Iron"),
        MyItemType.MakeOre("Silicon"),
        MyItemType.MakeOre("Nickel"),
        MyItemType.MakeOre("Cobalt"),
        MyItemType.MakeOre("Silver"),
        MyItemType.MakeOre("Platinum"),
        MyItemType.MakeOre("Magnesium"),
        MyItemType.MakeOre("Ice"),
        MyItemType.MakeOre("Uranium")
    };
private static MyItemType stone = MyItemType.MakeOre("Stone");

private int ticksRunning;
const String spinning = "-\\|/";

private enum GridStyle
    {
        SIMPLE,
        CORNER_SPIRAL
    }

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    registerState("detach", new StateDetach());
    registerState("travel_mine", new StateGoMine());
    registerState("dig", new StateDig());
    registerState("extract", new StateExtract());
    registerState("travel_home", new StateGoHome());
    registerState("dock", new StateDock());
    registerState("deposit", new StateDeposit());
    
    Me.CustomName = "Miner Control";
    model = new Drone(Me, this);
    
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid)
        {
            panel.CustomName = "Miner Info Screen";
            screen = panel;
            break;
        }
}

public void Save()
{
    Storage = string.Join(";", rotation%4, index, state, boreDepth);
    Storage += "|" + string.Join(";", (hasHomePos ? 1 : 0), homePos.X, homePos.Y, homePos.Z);
    Storage += "|" + string.Join(";", moveForw.X, moveForw.Y, moveForw.Z);
    Storage += "|" + string.Join(";", moveRight.X, moveRight.Y, moveRight.Z);
}

public void Main(string argument, UpdateType updateSource)
{
    hasClearedScreen = false;
    ++ticksRunning;
    echoToScreens("Termite OS Version "+version+" "+getSpinning()+"\n");
    echoToScreens("Borehole Index: "+index);
    
    if(argument.Length > 0)
        parseArg(argument);
    load(Storage);
    
    model.tick();
    if(!hasHomePos && model.isConnected())
        bindFromConnector(model);
    if(!model.runDiagnostic(echoToScreens) || model.isUnderControl() || index >= (GRID_SIZE * GRID_SIZE))
        setState(new String[]{"idle"});
    
    echoToScreens("Home position:");
    Vector3D home = getHomePosition();
    echoToScreens("  "+String.Join(", ",Math.Round(home.X,2), Math.Round(home.Y,2), Math.Round(home.Z,2)));
    
    echoToScreens("Mining direction:");
    Vector3D direct = Vector3D.Multiply(getDirection(rotation), STEP_DIST);
    echoToScreens("  "+String.Join(", ",Math.Round(direct.X,2), Math.Round(direct.Y,2), Math.Round(direct.Z,2)));
    
    echoToScreens("Current job ("+GRID_SIZE+"x"+GRID_SIZE+", "+boreDepth+"m)");
    Vector3D digSite = getMiningPosition();
    echoToScreens("  "+String.Join(", ",Math.Round(digSite.X,2), Math.Round(digSite.Y,2), Math.Round(digSite.Z,2)));
    echoToScreens("  Travel Distance "+Math.Round((home-digSite).Length(), 2));
    echoToScreens("");
    
    echoToScreens("State: "+state);
    if(ticksInvalid > 0)
        echoToScreens(" "+getSpinning()+" Changing State "+ticksInvalid);
    
    if(STATE_MAP.ContainsKey(state))
    {
        State currentState = STATE_MAP[state];
        currentState.tick(index, rotation, model, echoToScreens);
        if(currentState.invalid(model) && ticksInvalid++ >= 10)
        {
            setState(currentState.exit(model));
            ticksInvalid = 0;
            if(state == "deposit")
            {
                int oreCount = 0;
                foreach(var item in ores)
                    oreCount += model.getTotalOf(item);
                if(oreCount <= 10000)
                    index++;
            }
        }
    }
    else
        setState(new String[]{"idle"});
    
    echoToScreens("\nPower: "+Math.Round(model.getTotalChargePercent(),2)*100+"%");
    
    Save();
}

public void parseArg(String argument)
{
    String val = argument.ToLower();
    if(val == "reset")
        reset();
    else if(val == "remodel")
        model = new Drone(Me, this);
    else if(val == "turn")
        rotation = (rotation + 1)%4;
    else if(val.StartsWith("set_rotation="))
        rotation = int.Parse(val.Split('=')[1]) % 4;
    else if(val.StartsWith("set_index="))
        index = int.Parse(val.Split('=')[1]);
    else if(val == "set_home")
        bindFromConnector(model);
    else if(val.StartsWith("set_depth="))
        boreDepth = Math.Abs(int.Parse(val.Split('=')[1]));
    else if(val.StartsWith("set_state="))
        setState(new String[]{val.Split('=')[1]});
    
    Save();
}

public void load(String memory)
{
    reset();
    String[] lines = memory.ToLower().Split('|');
    int entries = lines.Length;
    
    // Operating data
    String[] data = lines[0].Split(';');
    if(data.Length == 4)
    {
        rotation = int.Parse(data[0]);
        index = int.Parse(data[1]);
        setState(new String[]{data[2]});
        boreDepth = int.Parse(data[3]);
    }
    if(--entries <= 0) return;
    
    // Home position
    data = lines[1].Split(';');
    if(data.Length == 4)
    {
        hasHomePos = int.Parse(data[0]) > 0;
        if(hasHomePos)
        {
            double x = double.Parse(data[1]);
            double y = double.Parse(data[2]);
            double z = double.Parse(data[3]);
            homePos = new Vector3D(x, y, z);
        }
    }
    if(--entries <= 0) return;
    
    // Forward vector
    data = lines[2].Split(';');
    if(data.Length == 3)
    {
        double x = double.Parse(data[0]);
        double y = double.Parse(data[1]);
        double z = double.Parse(data[2]);
        moveForw = new Vector3D(x, y, z);
    }
    if(--entries <= 0) return;
    
    // Right vector
    data = lines[3].Split(';');
    if(data.Length == 3)
    {
        double x = double.Parse(data[0]);
        double y = double.Parse(data[1]);
        double z = double.Parse(data[2]);
        moveRight = new Vector3D(x, y, z);
    }
    if(--entries <= 0) return;
}

public void reset()
{
    index = 0;
    rotation = 0;
    hasHomePos = false;
    homePos = new Vector3D(0,0,0);
    Save();
}

public char getSpinning()
{
    return spinning[this.ticksRunning % spinning.Length];
}

private static void registerState(String nameIn, State stateIn)
{
    if(nameIn == "" || stateIn == null)
        return;
    
    nameIn = nameIn.ToLower();
    STATE_MAP[nameIn] = stateIn;
}

// Exits the current state and moves to the first recognised one
private void setState(String[] statesIn)
{
    bool found = false;
    string enterState = state;
    foreach(String stateName in statesIn)
        if(STATE_MAP.ContainsKey(stateName.ToLower()))
        {
            enterState = stateName.ToLower();
            found = true;
            break;
        }
    if(!found)
        enterState = "idle";
    
    if(enterState != state)
    {
        if(STATE_MAP.ContainsKey(state))
            STATE_MAP[state].exit(model);
        
        state = "";
        Save();
        
        state = enterState;
        if(STATE_MAP.ContainsKey(state))
            STATE_MAP[state].enter(index, rotation, model);
    }
    
    Save();
}

public void echoToScreens(string text)
{
    Echo(text);
    if(screen != null && screen.IsFunctional && screen.Enabled)
        screen.WriteText((hasClearedScreen ? "\n" : "") + text, hasClearedScreen);
    hasClearedScreen = true;
}

public static bool hasHomePosition(){ return hasHomePos; }
public static Vector3D getHomePosition(){ return homePos; }
private static void setHomePosition(Vector3D vector)
{
    hasHomePos = true;
    homePos = vector;
}
public static void bindFromConnector(Drone model)
{
    setHomePosition(model.getWorldPosition());
    moveForw = Vector3D.TransformNormal(Vector3.Forward, model.getRemote().WorldMatrix);
    moveForw.Normalize();
    moveRight = Vector3D.TransformNormal(Vector3.Right, model.getRemote().WorldMatrix);
    moveRight.Normalize();
    rotation = 0;
}

public static Vector3D getDirection(int turns)
{
    Vector3D forward = moveForw;
    Vector3D right = moveRight;
    
    while(turns-- > 0)
    {
        Vector3D temp = Vector3D.Multiply(forward, -1);
        forward = right;
        right = temp;
    }
    return forward;
}

public static Vector3D getMiningPosition()
{
    Vector3D forward = getDirection(rotation);
    forward.Normalize();
    
    int row = 0;
    int col = 0;
    switch(STYLE)
    {
        case GridStyle.CORNER_SPIRAL:
            var layer = Math.Floor(Math.Sqrt(index));
            var indexInLayer = index - layer * (layer + 1);
            row = (int)(layer + Math.Min(indexInLayer, 0));
            col = (int)(layer - Math.Max(indexInLayer, 0));
            if(layer % 2 == 0) 
            {
                var tmp = row;
                row = col;
                col = tmp;
            }
            break;
        default:
            row = index&GRID_SIZE;
            col = MathHelper.FloorToInt(index/GRID_SIZE);
            break;
    }
    return getHomePosition() + Vector3D.Multiply(forward, STEP_DIST * row) + Vector3D.Multiply(getDirection(rotation + 1), STEP_DIST * col);
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

// A passable model of the drone running this program, only independent feature is gravity levelling
public class Drone
{
    private IMyCubeGrid localGrid;
    private IMyProgrammableBlock brain;
    private IMyRemoteControl remote;
    private IMyShipConnector connector;
    private List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    private List<IMyGyro> gyros = new List<IMyGyro>();
    private List<IMyShipDrill> drills = new List<IMyShipDrill>();
    private List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
    
    private List<IMyThrust> thrusters = new List<IMyThrust>();
    private Dictionary<Base6Directions.Direction, List<IMyThrust>> thrustMap = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
    private PID pidX = new PID(), pidY = new PID(), pidZ = new PID();
    private Vector3D targetVelocity = new Vector3D(0, 0, 0);
    
    Vector3D forwRef;
    Vector3D downRef;
    Vector3D gravTarget;
    
    public Drone(IMyProgrammableBlock me, MyGridProgram program)
    {
        localGrid = me.CubeGrid;
        brain = me;
        
        List<IMyRemoteControl> controls = new List<IMyRemoteControl>();
        program.GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(controls);
        foreach(var control in controls)
            if(control.CubeGrid == localGrid)
            {
                control.CustomName = "Miner Remote";
                remote = control;
                break;
            }
        
        List<IMyShipConnector> connectors = new List<IMyShipConnector>();
        program.GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
        foreach(var control in connectors)
            if(control.CubeGrid == localGrid)
            {
                control.CustomName = "Miner Connector";
                connector = control;
                break;
            }
        
        List<IMyCargoContainer> boxes = new List<IMyCargoContainer>();
        program.GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(boxes);
        foreach(var box in boxes)
            if(box.CubeGrid == localGrid)
            {
                box.CustomName = "Cargo "+cargo.Count();
                cargo.Add(box);
            }
        
        List<IMyGyro> gyroscopes = new List<IMyGyro>();
        program.GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyroscopes);
        foreach(var gyro in gyroscopes)
            if(gyro.CubeGrid == localGrid)
            {
                gyro.CustomName = "Gyroscope "+gyros.Count();
                gyro.ShowInTerminal = false;
                gyro.ShowInToolbarConfig = false;
                gyros.Add(gyro);
            }
        
        List<IMyShipDrill> drillers = new List<IMyShipDrill>();
        program.GridTerminalSystem.GetBlocksOfType<IMyShipDrill>(drillers);
        foreach(var box in drillers)
            if(box.CubeGrid == localGrid)
            {
                box.CustomName = "Drill "+drills.Count();
                box.ShowInTerminal = false;
                box.ShowInToolbarConfig = false;
                drills.Add(box);
            }
        
        List<IMyBatteryBlock> bats = new List<IMyBatteryBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMyBatteryBlock>(bats);
        foreach(var box in bats)
            if(box.CubeGrid == localGrid)
            {
                box.CustomName = "Battery "+batteries.Count();
                box.ShowInTerminal = false;
                box.ShowInToolbarConfig = false;
                batteries.Add(box);
            }
        
        foreach(var direction in Base6Directions.EnumDirections)
            thrustMap[direction] = new List<IMyThrust>();
        
        if(remote != null)
        {
            List<IMyThrust> thrusters2 = new List<IMyThrust>();
            program.GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters2);
            foreach(var thruster in thrusters2)
                if(thruster.CubeGrid == localGrid)
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
                        thruster.Enabled = true;
                        thruster.ThrustOverride = 0F;
                        thruster.CustomName = "Auxiliary Thruster";
                        continue;
                    }
                    
                    thrusters.Add(thruster);
                    registerThruster(thruster, localFace);
                    thruster.CustomName = "Thruster "+(Enum.GetName(typeof(Base6Directions.Direction), localFace)[0]+""+getThrusters(localFace).Count);
                }
        }
    }
    
    public void tick()
    {
        if(remote != null && remote.IsFunctional && remote.IsUnderControl)
        {
            resetGyros();
            resetThrusters(thrusters);
            return;
        }
        updateOrientationRef();
        updateLevelling();
        
        if(remote != null && remote.IsFunctional && !(isUnderControl() || isConnected()))
        {
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
    }
    
    public Vector3D toLocal(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, MatrixD.Transpose(remote.WorldMatrix)); }
    
    public bool runDiagnostic(Action<string> echo)
    {
        bool result = true;
        if(remote == null || !remote.IsFunctional)
        {
            echo(" X No operational remote control");
            result = false;
        }
        else if(!hasHomePosition())
        {
            echo(" X No home position set");
            result = false;
        }
        
        if(gyros.Count == 0)
        {
            echo(" X No gyroscopes");
            result = false;
        }
        else
        {
            bool functional = false;
            foreach(var gyro in gyros)
                if(gyro != null && gyro.IsFunctional && gyro.Enabled)
                {
                    functional = true;
                    break;
                }
            
            if(!functional)
            {
                echo(" X No gyroscopes functional & enabled");
                result = false;
            }
        }
        
        if(connector == null || !connector.IsFunctional)
        {
            echo(" X No functional connector");
            result = false;
        }
        
        if(drills.Count == 0)
        {
            echo(" X No drills");
            result = false;
        }
        else
        {
            bool functional = false;
            foreach(var drill in drills)
                if(drill != null && drill.IsFunctional)
                {
                    functional = true;
                    break;
                }
            
            if(!functional)
            {
                echo(" X No functional drills");
                result = false;
            }
        }
        
        if(cargo.Count == 0)
        {
            echo(" X No cargo containers");
            result = false;
        }
        else
        {
            bool functional = false;
            foreach(var box in cargo)
                if(box != null && box.IsFunctional)
                {
                    functional = true;
                    break;
                }
            
            if(!functional)
            {
                echo(" X  No functional cargo containers");
                result = false;
            }
        }
        
        if(batteries.Count == 0)
        {
            echo(" X No batteries");
            result = false;
        }
        else
        {
            bool functional = false;
            foreach(var bat in batteries)
                if(bat != null && bat.IsFunctional && bat.Enabled && bat.ChargeMode == ChargeMode.Auto)
                {
                    functional = true;
                    break;
                }
            
            if(!functional)
            {
                echo(" X No functional & enabled batteries providing power");
                result = false;
            }
        }
        
        return result;
    }
    
    public bool isUnderControl(){ return remote != null && remote.IsFunctional && remote.IsUnderControl; }
    
    public List<IMyCargoContainer> getCargo(){ return cargo; }
    public float getTotalMass(){ return remote == null ? 0F : remote.CalculateShipMass().TotalMass; }
    public float getMaxMass()
    {
        if(remote == null) return 0F;
        double gravity = getGravity().Length();
        
        float lift = 0F;
        foreach(var thruster in getThrusters(Base6Directions.Direction.Up))
            if(thruster != null && thruster.IsFunctional)
                lift += thruster.MaxThrust;
        
        return (float)(lift / gravity);
    }
    
    public bool isFull(){ return getTotalMass() >= getMaxMass(); }
    public bool isEmpty(){ return getCargoVolume() == 0F; }
    private float getCargoVolume()
    {
        float volume = 0F;
        foreach(var box in cargo)
        {
            if(box == null || !box.IsFunctional) continue;
            IMyInventory inv = box.GetInventory();
            volume += (float)inv.CurrentVolume.ToIntSafe();
        }
        return volume;
    }
    public bool hasAnyOf(MyItemType item)
    {
        foreach(var box in cargo)
            if(box.GetInventory().GetItemAmount(item).ToIntSafe() > 0)
                return true;
        return false;
    }
    public int getTotalOf(MyItemType item)
    {
        int tally = 0;
        foreach(var box in cargo)
            tally += box.GetInventory().GetItemAmount(item).ToIntSafe();
        return tally;
    }
    
    public void setDrills(bool on)
    {
        foreach(var drill in drills)
            if(drill != null && drill.IsFunctional)
                drill.Enabled = on;
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
    
    private void updateOrientationRef()
    {
        Matrix shipOrientation;
        remote.Orientation.GetMatrix(out shipOrientation);
        
        var quatPitch = Quaternion.CreateFromAxisAngle(shipOrientation.Left, 0F);
        var quatRoll = Quaternion.CreateFromAxisAngle(shipOrientation.Backward, 0F);
        forwRef = Vector3D.Transform(shipOrientation.Forward, quatPitch * quatRoll);
        forwRef.Y = 0;
        downRef = Vector3D.Transform(shipOrientation.Down, quatPitch * quatRoll);
        gravTarget = remote.GetNaturalGravity();
    }
    
    private void updateLevelling()
    {
        resetGyros();
        foreach(var gyro in gyros)
        {
            if(gyro == null || !gyro.Enabled || !gyro.IsFunctional) continue;
            Matrix localOrientation;
            gyro.Orientation.GetMatrix(out localOrientation);
            var axisPR = getGyroVec(gravTarget, downRef, gyro.WorldMatrix.GetOrientation(), localOrientation);
            
            gyro.GyroOverride = true;
            gyro.Pitch = (float)-axisPR.X;
            gyro.Roll = (float)-axisPR.Z;
        }
    }
    
    public void resetGyros()
    {
        foreach(var gyro in gyros)
        {
            if(gyro == null || !gyro.Enabled || !gyro.IsFunctional) continue;
            gyro.GyroOverride = false;
            gyro.Pitch = 0F;
            gyro.Yaw = 0F;
            gyro.Roll = 0F;
        }
    }
    
    public IMyRemoteControl getRemote(){ return remote; }
    public Vector3D getGravity(){ return remote == null ? new Vector3D(0, 0, 0) : remote.GetNaturalGravity(); }
    
    public Vector3D getWorldPosition(){ return connector.GetPosition(); }
    
    public IMyShipConnector getConnector(){ return connector; }
    public void setConnector(bool on){ connector.Enabled = on; }
    public bool isConnected(){ return connector.Status == MyShipConnectorStatus.Connected; }
    public MyShipConnectorStatus getConnectorStatus(){ return connector.Status; }
    public void toggleConnect(){ connector.ToggleConnect(); }
    
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

private abstract class State
{
    protected const double TOP_SPEED = 1D;
    
    private Controls[] affectedComponents = new Controls[0];
    
    protected State(Controls[] affectedIn){ affectedComponents = affectedIn; }
    
    // Performs any basic initial settings this state needs
    public virtual void enter(int index, int rotation, Drone model){ }
    // Performs the functions of this state
    public abstract void tick(int index, int rotation, Drone model, Action<string> echo);
    // Returns true if a state change should occur, ie. this state's work is finished and it can safely stop
    public abstract bool invalid(Drone model);
    // Resets any changes caused by this state and returns the state names that should be moved into
    public virtual String[] exit(Drone model)
    {
        resetModel(model);
        return exitStates(model);
    }
    public virtual void onExit(Drone model){ }
    public virtual String[] exitStates(Drone model){ return new String[]{"idle"}; }
    
    protected virtual void resetModel(Drone model)
    {
        foreach(var component in affectedComponents)
            switch(component)
            {
                case Controls.DRILLS: model.setDrills(false); break;
                case Controls.THRUSTERS: model.clearTargetVelocity(); break;
                case Controls.CONNECTOR: model.setConnector(true); break;
                case Controls.BATTERIES: model.resetBatteries(); break;
            }
    }
    
    protected enum Controls
    {
        DRILLS,
        THRUSTERS,
        CONNECTOR,
        BATTERIES
    }
}

private class StateDeposit : State
{
    public StateDeposit() : base(new Controls[]{Controls.BATTERIES}) { }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        model.doCharging();
        echo(" > Cargo Status:");
        foreach(var box in model.getCargo())
        {
            IMyInventory inv = box.GetInventory();
            double vol = (double)inv.CurrentVolume.ToIntSafe();
            double max = (double)inv.MaxVolume.ToIntSafe();
            echo("   * "+box.CustomName+": "+(int)(Math.Round(vol / max,2)*100)+"%");
        }
    }
    public override bool invalid(Drone model){ return model.isEmpty() && model.getTotalChargePercent() >= 0.8F; }
    public override String[] exitStates(Drone model){ return new String[]{"detach", "idle"}; }
}

// Detach from home connector and move away from it
private class StateDetach : State
{
    public StateDetach() : base(new Controls[]{Controls.CONNECTOR, Controls.THRUSTERS}) { }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        if(model.getTotalMass() > model.getMaxMass())
        {
            echo(" X Cannot safely undock, insufficient thrust for lift");
            return;
        }
        
        if(model.isConnected())
            model.toggleConnect();
        model.setConnector(false);
        
        if(distToHome(model) < 2D)
        {
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            model.setTargetVelocity(Vector3D.Multiply(gravity, TOP_SPEED));
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model)
    {
        return model.getConnectorStatus() == MyShipConnectorStatus.Unconnected && distToHome(model) >= 2D && model.getShipVelocities().Length() < 0.25D;
    }
    
    public override String[] exitStates(Drone model){ return new String[]{"travel_mine", "travel_home", "dock", "idle"}; }
    
    protected double distToHome(Drone model){ return (model.getWorldPosition() - getHomePosition()).Length(); }
}

// Move upwards towards home connector and dock with it
private class StateDock : StateDetach
{
    public StateDock(){ }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        echo(" > Connector: "+Enum.GetName(typeof(MyShipConnectorStatus), model.getConnectorStatus()));
        if(model.getConnectorStatus() == MyShipConnectorStatus.Connectable)
            model.toggleConnect();
        else if(distToHome(model) > 0.25D)
        {
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            model.setTargetVelocity(Vector3D.Multiply(gravity, -TOP_SPEED));
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model)
    {
        return model.isConnected();
    }
    
    public override String[] exitStates(Drone model)
    {
        if(!model.isConnected())
            return new String[]{"idle"};
        else
            return new String[]{"deposit", "detach", "travel_mine", "dig", "extract", "travel_home", "idle"};
    }
}

// Slowly descend whilst drills are on until cargo is full
private class StateDig : State
{
    bool isMining = false;
    Vector3D minePos;
    
    Vector3D digSite;
    
    public StateDig() : base(new Controls[]{Controls.THRUSTERS, Controls.DRILLS}) { }
    
    public bool canShipDig(Drone model){ return model.getTotalMass() < model.getMaxMass() && model.getTotalChargePercent() > 0.2F && (model.getWorldPosition()-digSite).Length() < boreDepth; }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        digSite = getMiningPosition();
        bool canDig = canShipDig(model);
        model.setDrills(canDig);
        bool empty = model.isEmpty();
        
        if(canDig)
        {
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            
            model.setTargetVelocity(Vector3D.Multiply(gravity, empty ? 0.5D : 0.025D));
            echo(" > Depth "+Math.Round((model.getWorldPosition()-digSite).Length(), 2)+" ("+(isMining ? Math.Round((model.getWorldPosition() - minePos).Length(),2) : 0)+")");
            echo(" > Lift Capacity : "+Math.Round(model.getTotalMass() / model.getMaxMass(),2)*100+"%");
            if(!isMining && !empty)
            {
                isMining = true;
                minePos = model.getWorldPosition();
            }
            else if(empty)
                isMining = false;
        }
        else
            model.clearTargetVelocity();
        
        if(!empty)
        {
            echo(" > Inventory");
            int stoneCount = model.getTotalOf(stone);
            int total = stoneCount;
            foreach(var item in ores)
                total += model.getTotalOf(item);
            
            foreach(var item in ores)
            {
                int count = model.getTotalOf(item);
                if(count > 0)
                    echo("   * "+item.SubtypeId+" "+count+" ("+Math.Round((float)count / (float)total, 2)*100+"%)");
            }
            if(stoneCount > 0)
                echo("   * "+stone.SubtypeId+" "+stoneCount+" ("+Math.Round((float)stoneCount / (float)total, 2)*100+"%)");
        }
    }
    
    public override bool invalid(Drone model){ return !canShipDig(model) && model.getShipVelocities().Length() < 0.1D; }
    
    public override String[] exitStates(Drone model){ return new String[]{"extract", "travel_home", "dock", "idle"}; }
}

// Base travelling state, moves the drone towards a destination laterally
private abstract class StateGo : State
{
    bool allowVertical;
    Vector3D destination;
    
    protected StateGo(bool verticalIn) : base(new Controls[]{Controls.THRUSTERS, Controls.CONNECTOR})
    {
        allowVertical = verticalIn;
    }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        destination = getDestination(index, rotation, model);
        Vector3D direction = (destination - model.getWorldPosition());
        double dist = distance(model);
        echo(" > Destination: "+String.Join(", ", Math.Round(destination.X,2), Math.Round(destination.Y,2), Math.Round(destination.Z,2)));
        echo(" > Direction: "+String.Join(", ", Math.Round(direction.X,2), Math.Round(direction.Y,2), Math.Round(direction.Z,2)));
        echo(" > Distance: "+Math.Round(dist,2));
        model.setConnector(false);
        if(dist > 0.5D)
        {
            Vector3D motion = direction;
            motion.Normalize();
            
            double speed = Math.Max(0.05D, Math.Min(TOP_SPEED, direction.Length()));
            model.setTargetVelocity(Vector3D.Multiply(motion, dist > 3D ? TOP_SPEED : 0.5D));
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model)
    {
        return distance(model) <= 0.5D && model.getShipVelocities().Length() < 0.25D;
    }
    
    protected abstract Vector3D getDestination(int index, int rotation, Drone model);
    
    private double distance(Drone model)
    {
        Vector3D direction = model.toLocal(destination - model.getWorldPosition());
        if(!allowVertical)
            direction.Y = 0D;
        return direction.Length();
    }
}

// Travel to calculated mining position, then dig
private class StateGoMine : StateGo
{
    public StateGoMine() : this(false){ }
    protected StateGoMine(bool verticalIn) : base(verticalIn){ }
    
    protected override Vector3D getDestination(int index, int rotation, Drone model)
    {
        Vector3D grav = model.getGravity();
        grav.Normalize();
        return getMiningPosition() + Vector3D.Multiply(grav, 2D);
    }
    
    public override String[] exitStates(Drone model){  return new String[]{"dig", "travel_home", "dock", "idle"}; }
}

// Travel from current position to home position
private class StateGoHome : StateGo
{
    public StateGoHome() : base(true){ }
    
    protected override Vector3D getDestination(int index, int rotation, Drone model)
    {
        Vector3D grav = model.getGravity();
        grav.Normalize();
        return getHomePosition() + Vector3D.Multiply(grav, 2D);
    }
    
    public override String[] exitStates(Drone model){ return new String[]{"dock", "idle"}; }
}

// Travel from current position to calculated mining position, with vertical thrust control permitted
private class StateExtract : StateGoMine
{
    public StateExtract() : base(true){ }
    
    public override String[] exitStates(Drone model){ return new String[]{"travel_home", "dock", "idle"}; }
}