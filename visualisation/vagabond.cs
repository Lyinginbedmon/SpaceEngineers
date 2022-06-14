// == Vagabond OS ==
const string version = "1.0";

// The Cockpit
IMyShipController cockpit;
// The Safe Zone
IMyFunctionalBlock safeZone;
// All batteries
List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
// All LCD panels
Dictionary<ScreenType, List<IMyTextPanel>> screenMap = new Dictionary<ScreenType, List<IMyTextPanel>>();
// All gas tanks
List<IMyGasTank> tanksH2 = new List<IMyGasTank>();
List<IMyGasTank> tanksO = new List<IMyGasTank>();
// Promenade Vent
IMyAirVent vent = null;
// All ship connectors
List<IMyShipConnector> connectors = new List<IMyShipConnector>();
Dictionary<Gantry, List<IMyShipConnector>> connectorMap = new Dictionary<Gantry, List<IMyShipConnector>>();
// All cargo containers
List<IMyTerminalBlock> containers = new List<IMyTerminalBlock>();
List<IMyProductionBlock> producers = new List<IMyProductionBlock>();

bool hasClearedScreen = false;
int ticksRunning = 0;

public enum Gantry
{
    AB,
    CD,
    EF,
    GH
}
public static Gantry[] gantries = new Gantry[]{Gantry.AB, Gantry.CD, Gantry.EF, Gantry.GH};

public enum ScreenType
{
    INFO,
    SHIPS,
    INV,
    BLANK
}
public static ScreenType[] screenTypes = new ScreenType[]{ScreenType.INFO, ScreenType.SHIPS, ScreenType.INV};

const String spinning = "-\\|/";

private static MyItemType[] resources = new MyItemType[]
    {
        MyItemType.MakeIngot("Iron"),
        MyItemType.MakeIngot("Silicon"),
        MyItemType.MakeIngot("Nickel"),
        MyItemType.MakeIngot("Cobalt"),
        MyItemType.MakeIngot("Silver"),
        MyItemType.MakeIngot("Platinum"),
        MyItemType.MakeIngot("Magnesium"),
        MyItemType.MakeOre("Ice"),
        MyItemType.MakeIngot("Uranium")
    };
private static MyItemType[] components = new MyItemType[]
    {
        MyItemType.MakeComponent("SteelPlate"),
        MyItemType.MakeComponent("InteriorPlate"),
        MyItemType.MakeComponent("Construction"),
        MyItemType.MakeComponent("MetalGrid"),
        MyItemType.MakeComponent("SmallTube"),
        MyItemType.MakeComponent("LargeTube"),
        MyItemType.MakeComponent("Motor"),
        MyItemType.MakeComponent("Computer"),
        MyItemType.MakeComponent("Display")
    };

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    
    Me.CustomName = "VBD System Control";
    List<IMyShipController> cockpits = new List<IMyShipController>();
    GridTerminalSystem.GetBlocksOfType<IMyShipController>(cockpits);
    foreach(var seat in cockpits)
        if(seat.CubeGrid == Me.CubeGrid && seat.IsMainCockpit)
        {
            seat.CustomName = "VBD Main Cockpit";
            cockpit = seat;
            break;
        }
    
    List<IMyBatteryBlock> bats = new List<IMyBatteryBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyBatteryBlock>(bats);
    foreach(var battery in bats)
        if(battery.CubeGrid == Me.CubeGrid)
        {
            battery.CustomName = "VBD Battery "+batteries.Count;
            battery.ShowInTerminal = false;
            battery.ShowInToolbarConfig = false;
            batteries.Add(battery);
        }
    
    List<IMyFunctionalBlock> zones = new List<IMyFunctionalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyFunctionalBlock>(zones);
    foreach(var zone in zones)
        if(zone.CubeGrid == Me.CubeGrid && zone.BlockDefinition.ToString().Contains("SafeZone"))
        {
            zone.CustomName = "VBD Safe Zone";
            safeZone = zone;
            break;
        }
    
    foreach(var type in screenTypes)
        screenMap[type] = new List<IMyTextPanel>();
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    int tally = 0;
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid)
        {
            panel.Enabled = true;
            panel.CustomName = "VBD Screen "+tally;
            panel.ShowInTerminal = false;
            panel.ShowInToolbarConfig = false;
            
            panel.FontColor = new Color(0, 0, 0);
            panel.BackgroundColor = new Color(0, 125, 255);
            
            ScreenType type = getScreenFromString(panel.CustomData);
            if(type != ScreenType.BLANK)
            {
                panel.ContentType = ContentType.TEXT_AND_IMAGE;
                switch(type)
                {
                    case ScreenType.INFO:    panel.CustomName = "VBD Info Screen "+screenMap[type].Count; break;
                    case ScreenType.SHIPS:    panel.CustomName = "VBD Docking Screen "+screenMap[type].Count; break;
                    case ScreenType.INV:    panel.CustomName = "VBD Inventory Screen "+screenMap[type].Count; break;
                }
                screenMap[type].Add(panel);
            }
            else
                ++tally;
        }
    
    List<IMyGasTank> tanks = new List<IMyGasTank>();
    GridTerminalSystem.GetBlocksOfType<IMyGasTank>(tanks);
    foreach(var tank in tanks)
        if(tank.CubeGrid == Me.CubeGrid)
        {
            String type = tank.BlockDefinition.ToString().ToLower();
            tank.ShowInTerminal = false;
            tank.ShowInToolbarConfig = false;
            if(type.Contains("hydrogen"))
            {
                tank.CustomName = "VBD Hydrogen Tank "+tanksH2.Count;
                tanksH2.Add(tank);
            }
            else
            {
                tank.CustomName = "VBD Oxygen Tank "+tanksO.Count;
                tanksO.Add(tank);
            }
        }
    
    if(cockpit != null)
    {
        List<IMyAirVent> vents = new List<IMyAirVent>();
        GridTerminalSystem.GetBlocksOfType<IMyAirVent>(vents);
        double closest = Double.MaxValue;
        foreach(var vent2 in vents)
            if(vent2.CubeGrid == Me.CubeGrid)
            {
                double dist = (vent2.GetPosition() - cockpit.GetPosition()).Length();
                if(dist < closest)
                {
                    vent = vent2;
                    closest = dist;
                }
            }
    }
    
    foreach(var gantry in gantries)
        connectorMap[gantry] = new List<IMyShipConnector>();
    Vector3D refWorldPos = cockpit == null ? new Vector3D(0,0,0) : cockpit.WorldMatrix.Translation;
    List<IMyShipConnector> connectors2 = new List<IMyShipConnector>();
    GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors2);
    foreach(var connector in connectors2)
        if(connector.CubeGrid == Me.CubeGrid)
        {
            connector.CustomName = "VBD Dock "+connectors.Count;
            connector.ShowInTerminal = false;
            connector.ShowInToolbarConfig = false;
            connectors.Add(connector);
            containers.Add(connector);
            
            if(cockpit != null)
            {
                Vector3D pos = connector.GetPosition();
                Vector3D worldDirection = pos - refWorldPos;
                Vector3D localPos = Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(cockpit.WorldMatrix));
                Base6Directions.Direction face = Base6Directions.Direction.Forward;
                if(Math.Abs(localPos.X) > Math.Abs(localPos.Z))
                    face = localPos.X > 0 ? Base6Directions.Direction.Left : Base6Directions.Direction.Right;
                else
                    face = localPos.Z > 0 ? Base6Directions.Direction.Backward : Base6Directions.Direction.Forward;
                Gantry gantry = getGantryFromDirection(face);
                connectorMap[gantry].Add(connector);
            }
        }
    
    foreach(var gantry in gantries)
    {
        List<IMyShipConnector> connectors = sortByDistance(connectorMap[gantry]);
        int index = 1;
        foreach(var connector in connectors)
            connector.CustomName = Enum.GetName(typeof(Gantry), gantry) + " Dock "+(index++);
        connectorMap[gantry] = connectors;
    }
    
    List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(cargo);
    int boxes = 0;
    foreach(var box in cargo)
        if(box.CubeGrid == Me.CubeGrid)
        {
            box.CustomName = "VBD Cargo Container "+(boxes++);
            box.ShowInTerminal = false;
            box.ShowInToolbarConfig = false;
        }
    
    List<IMyTerminalBlock> terminals = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(terminals);
    foreach(var ass in terminals)
        if(ass.CubeGrid == Me.CubeGrid && ass.HasInventory)
            containers.Add(ass);
    
    List<IMyProductionBlock> production = new List<IMyProductionBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyProductionBlock>(production);
    foreach(var prod in production)
        if(prod.CubeGrid == Me.CubeGrid)
            producers.Add(prod);
}

public void Main(string argument, UpdateType updateSource)
{
    ++ticksRunning;
    handleInfoScreens(screenMap[ScreenType.INFO]);
    handleShipScreens(screenMap[ScreenType.SHIPS]);
    handleInvScreens(screenMap[ScreenType.INV]);
}

public void handleInfoScreens(List<IMyTextPanel> screens)
{
    hasClearedScreen = false;
    
    echoToScreens(screens, "Vagabond OS Version "+version+" "+getSpinning()+"\n");
    echoToScreens(screens, "Safe Zone is "+(safeZone == null ? "ABSENT" : (safeZone.Enabled ? "ACTIVE" : "INACTIVE")));
    
    if(vent != null)
    {
        switch(vent.Status)
        {
            case VentStatus.Pressurized: echoToScreens(screens, "Promenade is PRESSURISED\n"); break;
            case VentStatus.Pressurizing: echoToScreens(screens, "Promenade is PRESSURISING...\n"); break;
            default:    echoToScreens(screens, "Promenade is UNPRESSURISED!\n"); break;
        }
    }
    
    // Battery power
    float usage = 0F;
    float maxUsage = 0F;
    float input = 0F;
    float maxInput = 0F;
    float capacity = 0F;
    foreach(var bat in batteries)
    {
        usage += bat.CurrentOutput;
        maxUsage += bat.MaxOutput;
        
        input += Math.Max(0F, bat.CurrentInput);
        maxInput += bat.MaxInput;
        capacity += bat.MaxStoredPower;
    }
    echoToScreens(screens, "Available Power: "+Math.Round(getTotalChargePercent()*capacity, 2)+"MWh");
    echoToScreens(screens, percentToBar(getTotalChargePercent()));
    echoToScreens(screens, " > In    "+percentToBar(input/maxInput, 0.03F)+" "+Math.Round(input,2)+"MW");
    echoToScreens(screens, " > Out "+percentToBar(usage/maxUsage, 0.03F)+" "+Math.Round(usage,2)+"MW");
    
    // Gas tanks
    float hydrogen = getTotalGasPercent(tanksH2);
    capacity = 0F;
    foreach(var tank in tanksH2)
        capacity += tank.Capacity;
    echoToScreens(screens, "\nHydrogen: "+Math.Round(hydrogen*capacity, 2)+"L");
    echoToScreens(screens, percentToBar(hydrogen));
    
    float oxygen = getTotalGasPercent(tanksO);
    capacity = 0F;
    foreach(var tank in tanksO)
        capacity += tank.Capacity;
    echoToScreens(screens, "Oxygen: "+Math.Round(oxygen*capacity, 2)+"L");
    echoToScreens(screens, percentToBar(oxygen));
}

public void handleShipScreens(List<IMyTextPanel> screens)
{
    hasClearedScreen = false;
    
    int tally = 0;
    foreach(var connector in connectors)
        if(connector.Status == MyShipConnectorStatus.Connected)
            tally++;
    
    int pageRate = 20;
    int index = (ticksRunning / pageRate) % gantries.Length;
    echoToScreens(screens, tally+" Docked Ships:");
    
    Gantry current = gantries[index];
    String page = " > Gantry ";
    foreach(var gantry in gantries)
    {
        String name = Enum.GetName(typeof(Gantry), gantry);
        page += (gantry == current ? "["+name+"]" : name) + " ";
    }
    echoToScreens(screens, page);
    
    int indices = 1;
    foreach(var connector in connectorMap[gantries[index]])
    {
        String status = "    > "+(indices++)+": ";
        switch(connector.Status)
        {
            case MyShipConnectorStatus.Connected:
                status += connector.OtherConnector.CubeGrid.CustomName;
                break;
            case MyShipConnectorStatus.Connectable:
                status += getSpinning()+" Docking In Progress "+getSpinning();
                break;
            default:
                break;
        }
        echoToScreens(screens, status);
    }
}

public void handleInvScreens(List<IMyTextPanel> screens)
{
    hasClearedScreen = false;
    echoToScreens(screens, "Available Resources:");
    Dictionary<MyItemType, int> itemMap = new Dictionary<MyItemType, int>();
    int longestName = 0;
    MyItemType[] listedItems = (ticksRunning/30)%2 == 0 ? resources : components;
    foreach(var item in listedItems)
    {
        itemMap[item] = getTotalOfItem(item);
        longestName = Math.Max(longestName, item.SubtypeId.Length);
    }
    foreach(var item in listedItems)
    {
        String entry = " > "+item.SubtypeId + ":";
        float tally = (float)itemMap[item];
        String suffix = "";
        if(tally > 1000000)
        {
            tally /= 1000000;
            suffix = "m";
        }
        else if(tally > 1000)
        {
            tally /= 1000;
            suffix = "k";
        }
        String vol = tally == 0F ? "-" : Math.Round(tally,2) + suffix;
        entry = entry.PadRight(60 - entry.Length - vol.Length);
        echoToScreens(screens, entry+vol);
    }
        
    int volume = 0;
    int capacity = 0;
    foreach(var box in containers)
    {
        volume += box.GetInventory().CurrentVolume.ToIntSafe();
        capacity += box.GetInventory().MaxVolume.ToIntSafe();
    }
    foreach(var box in producers)
    {
        volume += box.OutputInventory.CurrentVolume.ToIntSafe();
        capacity += box.OutputInventory.MaxVolume.ToIntSafe();
    }
    echoToScreens(screens, "\nCargo Volume: "+volume+"000L");
    echoToScreens(screens, percentToBar((float)volume / (float)capacity));
}

// #### UTILITY FUNCTIONS ####

public char getSpinning()
{
    return spinning[this.ticksRunning % spinning.Length];
}

public ScreenType getScreenFromString(String value)
{
    foreach(var type in screenTypes)
        if(Enum.GetName(typeof(ScreenType), type).ToLower() == value.ToLower())
            return type;
    return ScreenType.BLANK;
}

public Gantry getGantryFromDirection(Base6Directions.Direction face)
{
    switch(face)
    {
        case Base6Directions.Direction.Forward:    return Gantry.CD;
        case Base6Directions.Direction.Backward:    return Gantry.GH;
        case Base6Directions.Direction.Right:    return Gantry.AB;
        case Base6Directions.Direction.Left:    return Gantry.EF;
        default:    return Gantry.AB;
    }
}

public List<IMyShipConnector> sortByDistance(List<IMyShipConnector> list)
{
    List<IMyShipConnector> sorted = new List<IMyShipConnector>();
    while(list.Count > 0)
    {
        if(cockpit == null)
        {
            sorted.AddRange(list);
            list.Clear();
            continue;
        }
        IMyShipConnector closest = null;
        double distance = Double.MaxValue;
        foreach(var connector in list)
        {
            double dist = (connector.GetPosition() - cockpit.GetPosition()).Length();
            if(dist <= distance)
            {
                distance = dist;
                closest = connector;
            }
        }
        sorted.Add(closest);
        list.Remove(closest);
    }
    return sorted;
}

public string percentToBar(float percentage, float divisor)
{
    percentage = clamp(percentage, 0F, 1F);
    String bar = "[";
    float val = 0F;
    while(val < 1F)
    {
        bar += val <= percentage ? "|" : "'";
        val += divisor;
    }
    bar += "]";
    return bar;
}

public string percentToBar(float percentage)
{
    return percentToBar(percentage, 0.0125F);
}

public float getTotalChargePercent()
{
    float charge = 0F;
    float maxCharge = 0F;
    foreach(var battery in batteries)
        if(battery.ChargeMode != ChargeMode.Recharge && battery.IsFunctional && battery.Enabled)
        {
            charge += battery.CurrentStoredPower;
            maxCharge += battery.MaxStoredPower;
        }
    
    return charge / maxCharge;
}

public void echoToScreens(List<IMyTextPanel> screens, string text)
{
    Echo(text);
    addStringToDisplays(screens, text, hasClearedScreen);
    hasClearedScreen = true;
}

public void addStringToDisplays(List<IMyTextPanel> displays, string text, bool append)
{
    foreach(var screen in displays)
        screen.WriteText((append ? "\n" : "") + text, append);
}

private float clamp(float val, float min, float max)
{
    float minVal = Math.Min(min, max);
    float maxVal = Math.Max(min, max);
    return Math.Max(minVal, Math.Min(maxVal, val));
}

public float getTotalGasPercent(List<IMyGasTank> tanks)
{
    double total = 0D;
    double max = 0D;
    foreach(var tank in tanks)
    {
        total += tank.FilledRatio * tank.Capacity;
        max += tank.Capacity;
    }
    return (float)(total / max);
}

public int getTotalOfItem(MyItemType item)
{
    int tally = 0;
    foreach(var box in containers) tally += box.GetInventory().GetItemAmount(item).ToIntSafe();
    foreach(var box in producers) tally += box.OutputInventory.GetItemAmount(item).ToIntSafe();
    return tally;
}