// == Shine, Bright Morning Light OS ==
const string version = "1.0";

// Block references
List<IMyGyro> gyros = new List<IMyGyro>();
List<IMyThrust> cruiseThrusters = new List<IMyThrust>();
List<IMyThrust> reverseThrusters = new List<IMyThrust>();
IMyShipController cockpit;

// Gyro control values
Vector3D reference;
Vector3D target;
double gravity;

int cruisingSpeed = 40;
bool cruise = false;
bool cruiseControl = false;
PID cruisePid = new PID();

private int ticksRunning;
const String spinning = "-\\|/";

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    
    	Me.CustomName = "SBML System Control";
	   
    List<IMyGyro> gyros2 = new List<IMyGyro>();
    GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyros2);
    foreach(var gyro in gyros2)
        if(gyro.CubeGrid == Me.CubeGrid)
        {
            gyro.CustomName = "SBML Gyro "+gyros.Count;
            gyros.Add(gyro);
        }
    
    List<IMyShipController> cockpits = new List<IMyShipController>();
    GridTerminalSystem.GetBlocksOfType<IMyShipController>(cockpits);
    foreach(var seat in cockpits)
        if(seat.CubeGrid == Me.CubeGrid && seat.IsMainCockpit)
        {
            seat.CustomName = "SBML Cockpit";
            cockpit = seat;
        }
    
    var forward = cockpit.Orientation.TransformDirection(Base6Directions.Direction.Forward);
    var backward = cockpit.Orientation.TransformDirection(Base6Directions.Direction.Backward);
    
    List<IMyThrust> thrusters = new List<IMyThrust>();
    GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters);
    foreach(var thruster in thrusters)
        if(thruster.CubeGrid == Me.CubeGrid)
            if(thruster.Orientation.TransformDirection(Base6Directions.Direction.Forward) == backward)
                cruiseThrusters.Add(thruster);
            else if(thruster.Orientation.TransformDirection(Base6Directions.Direction.Forward) == forward)
                reverseThrusters.Add(thruster);
    
    resetGyros();
}

public void Save()
{
    Storage = String.Join(";",cruise ? 1 : 0, cruisingSpeed);
}

public void Main(string argument, UpdateType updateSource)
{
    ++ticksRunning;
    echoToScreens("SBML Control OS Version "+version+" "+getSpinning());
    if(cockpit == null)
    {
        echoToScreens("No main cockpit found");
        return;
    }
    echoToScreens("Main Cockpit: "+cockpit.CustomName);
    echoToScreens("");
    if(argument.Length > 0)
    {
        var parts = argument.Split(';');
        parseCruiseSpeed(parts[0]);
        Save();
    }
    else if(Storage.Length > 0)
    {
        var parts = Storage.Split(';');
        cruise = int.Parse(parts[0]) > 0;
        cruisingSpeed = int.Parse(parts[1]);
    }
    
    handleCruiseControl();
    handleGyros(cockpit.RollIndicator);
}

private void parseCruiseSpeed(string argument)
{
    String[] data = argument.Split(';');
    for(int i=0; i<data.Length; i++)
    {
        String entry = data[i].ToLower();
        if(entry == "cruise")
            cruise = true;
        else if(entry == "stop")
            cruise = false;
        else
            try
            {
                cruisingSpeed = Math.Max(0, Math.Min(60, int.Parse(entry)));
            }
            catch(Exception e){ }
    }
}

public char getSpinning()
{
    return spinning[this.ticksRunning % spinning.Length];
}

public void echoToScreens(string text)
{
    Echo(text);
}

private void handleCruiseControl()
{
    bool oldCruise = cruiseControl;
    
    bool hasThruster = false;
    foreach(var thruster in cruiseThrusters)
        if(thruster != null && thruster.IsFunctional)
            hasThruster = true;
    
    bool hasThruster2 = false;
    foreach(var thruster in reverseThrusters)
        if(thruster != null && thruster.IsFunctional)
            hasThruster2 = true;
    
    cruiseControl = hasThruster && hasThruster2 && cruisingSpeed > 0 && cruise;
    
    echoToScreens("Cruise Control: "+(cruiseControl ? "ON" : "OFF"));
    echoToScreens(" > Set Speed: "+cruisingSpeed);
    if(cruisingSpeed <= 0)
        echoToScreens(" X Cruising speed set too low");
    if(!hasThruster)
        echoToScreens(" X No forward thruster available");
    if(!hasThruster2)
        echoToScreens(" X No reverse thruster available");
    if(cruise != cruiseControl)
        cruise = cruiseControl;
    
    // If cruise control state has changed...
    if(oldCruise != cruiseControl)
    {
        resetGyros();
        cockpit.SetValueBool("ControlGyros", !cruiseControl);
        if(!cruiseControl)
            cockpit.DampenersOverride = true;
    }
    
    // In cruise control, disable rotation control and maintain cruising speed efficiently
    if(cruiseControl)
    {
        Vector3D velocity = Vector3D.TransformNormal(cockpit.GetShipVelocities().LinearVelocity, MatrixD.Transpose(cockpit.WorldMatrix));
        cockpit.SetValueBool("ControlGyros", false);
        echoToScreens(" > Current Speed: "+(int)Math.Abs(velocity.Z));
        
        double mass = cockpit.CalculateShipMass().TotalMass;
        handleThrust(cruisingSpeed, -velocity.Z, cruisePid, mass, cruiseThrusters, reverseThrusters);
    }
    else
    {
        resetThrusters(cruiseThrusters);
        resetThrusters(reverseThrusters);
    }
}

public void handleThrust(double target, double current, PID pid, double mass, List<IMyThrust> groupA, List<IMyThrust> groupB)
{
    handleThrust(target, current, pid, mass, groupA, groupB, 0D);
}

public void handleThrust(double target, double current, PID pid, double mass, List<IMyThrust> groupA, List<IMyThrust> groupB, double offset)
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

public void resetThrusters(List<IMyThrust> list)
{
    foreach(var thruster in list)
    {
        if(thruster == null) continue;
        thruster.Enabled = true;
        thruster.ThrustOverride = 0F;
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

private void handleGyros(float roll)
{
    echoToScreens("Gyroscopes:");
    updateOrientationRef();
    int controlled = 0;
    int manual = 0;
    resetGyros();
    
    int gyroNum = 0;
    foreach(var gyro in gyros)
    {
        if(gyro.CubeGrid != Me.CubeGrid)
            continue;
        
        if(gyroNum++ % (cruiseControl ? 8 : 2) == 0)
        {
            if(roll != 0)
            {
                gyro.GyroOverride = true;
                gyro.Yaw = 0F;
                gyro.Roll = -roll;
            }
            manual++;
            continue;
        }
        else
            controlled++;
        
        Matrix localOrientation;
        gyro.Orientation.GetMatrix(out localOrientation);
        
        var localReference = Vector3D.Transform(reference, MatrixD.Transpose(localOrientation));
        var localTarget = Vector3D.Transform(target, MatrixD.Transpose(gyro.WorldMatrix.GetOrientation()));
        
        var axis = Vector3D.Cross(localReference, localTarget);
        var angle = axis.Length();
        angle = Math.Atan2(angle, Math.Sqrt(Math.Max(0, 1 - angle * angle)));
        if(Vector3D.Dot(localReference, localTarget) < 0)
            angle = Math.PI;
        
        gyro.GyroOverride = true;
        gyro.Pitch = (float)-axis.X;
        gyro.Yaw = -roll;
        gyro.Roll = (float)-axis.Z;
    }
    echoToScreens(" "+controlled+" controlled");
    echoToScreens(" "+manual+" manual");
}

private void resetGyros()
{
    foreach(var gyro in gyros)
    {
        gyro.GyroOverride = false;
        gyro.Roll = 0F;
        gyro.Yaw = 0F;
        gyro.Pitch = 0F;
    }
}

private void updateOrientationRef()
{
    Matrix cockpitOrientation;
    cockpit.Orientation.GetMatrix(out cockpitOrientation);
    
    var quatPitch = Quaternion.CreateFromAxisAngle(cockpitOrientation.Left, 0F);
    var quatRoll = Quaternion.CreateFromAxisAngle(cockpitOrientation.Backward, 0F);
    var orientation = Vector3D.Transform(cockpitOrientation.Down, quatPitch * quatRoll);
    
    reference = orientation;
    target = cockpit.GetNaturalGravity();
    gravity = target.Length() / 10;
}