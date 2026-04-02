// == Creation Crafting Management OS ==
const String version = "1.0";
const int INT_MAX = 2147483647;
const String spinning = "-\\|/";
const String config_default = 
    "[general]\nPrefix=CRT\nCraftRate=10\n\n[component_a_thresholds]\nSteelPlate=100000\nInteriorPlate=100000\nConstruction=50000\nSmallTube=10000\nLargeTube=10000\nGirder=10000\n\n[component_b_thresholds]\nMetalGrid=1000\nSolarCell=1000\nDisplay=1000\nPowerCell=1000\nMotor=1000\nComputer=1000\n\n[prototech_thresholds]\nPrototechCapacitor=100\nPrototechCircuitry=100\nPrototechCoolingUnit=100\nPrototechFrame=20\nPrototechMachinery=100\nPrototechPanel=200\nPrototechPropulsionUnit=100\nPrototechScrap=0";

String prefix;
int craftingRate;
// All LCD panels
Dictionary<ScreenType, List<IMyTextPanel>> screenMap = new Dictionary<ScreenType, List<IMyTextPanel>>();
// All cargo containers
List<IMyTerminalBlock> containers = new List<IMyTerminalBlock>();
// All assemblers
List<IMyAssembler> assemblers = new List<IMyAssembler>();

bool hasClearedScreen = false;
int ticksRunning = 0;

public enum ScreenType
{
    COMP_0,
    COMP_1,
    PROTO,
    RESOURCES,
    BLANK
}
public static ScreenType[] screenTypes = new ScreenType[]{ScreenType.COMP_0, ScreenType.COMP_1, ScreenType.PROTO, ScreenType.RESOURCES};

// List of all items recognised by this system
private static List<MyItemType> monitoredComponents = new List<MyItemType>();

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
private static MyItemType[] components_0 = new MyItemType[]
    {
        MyItemType.MakeComponent("SteelPlate"),
        MyItemType.MakeComponent("InteriorPlate"),
        MyItemType.MakeComponent("Construction"),
        MyItemType.MakeComponent("SmallTube"),
        MyItemType.MakeComponent("LargeTube"),
        MyItemType.MakeComponent("Girder")
    };
private static MyItemType[] components_1 = new MyItemType[]
    {
        MyItemType.MakeComponent("MetalGrid"),
        MyItemType.MakeComponent("SolarCell"),
        MyItemType.MakeComponent("Display"),
        MyItemType.MakeComponent("PowerCell"),
        MyItemType.MakeComponent("Motor"),
        MyItemType.MakeComponent("Computer")
    };
private static MyItemType[] components_p = new MyItemType[]
    {
        MyItemType.MakeComponent("PrototechCapacitor"),
        MyItemType.MakeComponent("PrototechCircuitry"),
        MyItemType.MakeComponent("PrototechCoolingUnit"),
        MyItemType.MakeComponent("PrototechFrame"),
        MyItemType.MakeComponent("PrototechMachinery"),
        MyItemType.MakeComponent("PrototechPanel"),
        MyItemType.MakeComponent("PrototechPropulsionUnit"),
        MyItemType.MakeComponent("PrototechScrap")
    };
// FIXME Known bug: system fails to identify PrototechScrap in inventories, resulting in constant crafting while threshold > 0, only item with this issue

// Dictionary of monitored components to their configured crafting thresholds
private static Dictionary<MyItemType,int> componentThresholds = new Dictionary<MyItemType, int>();
// Dictionary of monitored components to how many are needed to meet thresholds
private static Dictionary<MyItemType,int> componentRequests = new Dictionary<MyItemType, int>();

private static MyIni config = new MyIni();

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    
    // Define component thresholds
    loadConfig(Me.CustomData);
    
    Me.CustomName = prefix+" System Control";
    
    // Assess monitored components
    foreach(MyItemType item in components_0)
        monitoredComponents.Add(item);
    foreach(MyItemType item in components_1)
        monitoredComponents.Add(item);
    foreach(MyItemType item in components_p)
        monitoredComponents.Add(item);
    foreach(MyItemType item in resources)
        monitoredComponents.Add(item);
    
    // Identify display screens
    foreach(var type in screenTypes)
        screenMap[type] = new List<IMyTextPanel>();
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid)
        {
            panel.ShowInTerminal = false;
            panel.ShowInToolbarConfig = false;
            
            ScreenType type = getScreenFromString(panel.CustomData);
            if(type != ScreenType.BLANK)
            {
                panel.ContentType = ContentType.TEXT_AND_IMAGE;
                switch(type)
                {
                    case ScreenType.COMP_0:    panel.CustomName = prefix+" Comp. A Inv Screen "+screenMap[type].Count; break;
                    case ScreenType.COMP_1:    panel.CustomName = prefix+" Comp. B Inv Screen "+screenMap[type].Count; break;
                    case ScreenType.PROTO:    panel.CustomName = prefix+" Prototech Inv Screen "+screenMap[type].Count; break;
                    case ScreenType.RESOURCES:    panel.CustomName = prefix+" Resources Screen "+screenMap[type].Count; break;
                }
                screenMap[type].Add(panel);
            }
        }
    
    // Identify monitored cargo inventories
    List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(cargo);
    int boxes = 0;
    foreach(var box in cargo)
        if(box.CubeGrid == Me.CubeGrid && box.CustomData.ToLower() == "main_hold")
        {
            box.CustomName = prefix+" Cargo Hold "+(++boxes);
            box.ShowInTerminal = false;
            box.ShowInToolbarConfig = false;
            containers.Add(box);
        }
    
    // Identify nominated assemblers
    List<IMyAssembler> crafters = new List<IMyAssembler>();
    GridTerminalSystem.GetBlocksOfType<IMyAssembler>(crafters);
    boxes = 0;
    foreach(var ass in crafters)
        if(ass.CubeGrid == Me.CubeGrid && ass.CustomData.ToLower() == "auto_assembler")
        {
            string type = "Assembler";
            string subtype = ass.BlockDefinition.SubtypeName;
            if(subtype.Contains("Prototech"))
                type = "Prototech Assembler";
            else if(subtype.Contains("Basic"))
                type = "Basic Assembler";
            // Never use survival kits for autocrafting
            else if(subtype.Contains("Survival"))
                continue;
            
            ass.CustomName = prefix+" Managed "+type+" "+(++boxes);
            ass.ShowInTerminal = false;
            ass.ShowInToolbarConfig = false;
            assemblers.Add(ass);
        }
}

public void loadConfig(String customData)
{
    if(customData.Length == 0)
    {
        Me.CustomData = config_default;
        customData = Me.CustomData;
    }
    
    MyIniParseResult result;
    if(!config.TryParse(customData, out result))
    {
        Runtime.UpdateFrequency = UpdateFrequency.Once;
        throw new Exception(result.ToString());
    }
    
    prefix = config.Get("general", "Prefix").ToString("CRT");
    craftingRate = config.Get("general", "CraftRate").ToInt32(10);
    
    foreach(MyItemType item in components_0)
        componentThresholds[item] = config.Get("component_a_thresholds", item.SubtypeId).ToInt32(0);
    
    foreach(MyItemType item in components_1)
        componentThresholds[item] = config.Get("component_b_thresholds", item.SubtypeId).ToInt32(0);
    
    foreach(MyItemType item in components_p)
        componentThresholds[item] = config.Get("prototech_thresholds", item.SubtypeId).ToInt32(0);
}

public void Main(string argument, UpdateType updateSource)
{
    ++ticksRunning;
    componentRequests.Clear();
    handleInvScreens(screenMap[ScreenType.COMP_0], components_0);
    handleInvScreens(screenMap[ScreenType.COMP_1], components_1);
    handleInvScreens(screenMap[ScreenType.PROTO], components_p);
    handleResScreens();
    
    if(ticksRunning%craftingRate == 0)
        updateAutoCrafting();
}

// #### MANAGEMENT FUNCTIONS ####

public void handleResScreens()
{
    List<IMyTextPanel> screens = screenMap[ScreenType.RESOURCES];
    MyItemType[] listedItems = resources;
    hasClearedScreen = false;
    echoToScreens(screens, "Available Resources:");
    Dictionary<MyItemType, int> itemMap = new Dictionary<MyItemType, int>();
    int longestName = 0;
    foreach(var item in listedItems)
    {
        itemMap[item] = getTotalOfItem(item);
        longestName = Math.Max(longestName, item.SubtypeId.Length);
    }
    foreach(var item in listedItems)
    {
        String name = item.SubtypeId;
        String entry = " > "+name+": ";
        if(name.Length < 30)
            entry = entry.PadRight(30 - name.Length);
        entry += abbreviateValue(itemMap[item]);
        echoToScreens(screens, entry);
    }
}

public void handleInvScreens(List<IMyTextPanel> screens, MyItemType[] listedItems)
{
    hasClearedScreen = false;
    echoToScreens(screens, "Available Components:");
    String entry = "".PadRight(20);
    foreach(var item in listedItems)
    {
        echoToScreens(screens, " > "+item.SubtypeId + ":");
        int tally = getTotalOfItem(item);
        int threshold = componentThresholds[item];
        if(threshold <= 0)
            echoToScreens(screens, entry+abbreviateValue(tally));
        else
        {
            String vol = abbreviateValue(tally) + " / " + abbreviateValue(threshold);
            if(threshold > 0 && tally < threshold)
            {
                vol += " "+getSpinning();
                componentRequests[item] = threshold - tally;
            }
            echoToScreens(screens, entry+vol);
        }
    }
}

public void updateAutoCrafting()
{
    // Dictionary of items currently queued up in monitored assemblers
    Dictionary<MyItemType,int> craftingQueue = new Dictionary<MyItemType,int>();
    
    foreach(MyItemType product in componentRequests.Keys)
    {
        int tally = 0;
        foreach(IMyAssembler assembler in assemblers)
        {
            List<MyProductionItem> queue = new List<MyProductionItem>();
            assembler.GetQueue(queue);
            foreach(MyProductionItem recipe in queue)
                if(isRequestOfItem(recipe, product))
                    tally += recipe.Amount.ToIntSafe();
        }
        
        if(tally < componentRequests[product])
            craftingQueue[product] = componentRequests[product] - tally;
    }
    
    Echo("Latest autocrafting report:");
    foreach(MyItemType item in craftingQueue.Keys)
    {
        Echo("> "+craftingQueue[item]+"x "+item.SubtypeId);
        pushRequest(createRequest(item), craftingQueue[item]);
    }
}

// #### UTILITY FUNCTIONS ####

public char getSpinning()
{
    return spinning[this.ticksRunning % spinning.Length];
}

public string abbreviateValue(int value)
{
    if(value == 0F)
        return "-";
    else if(value >= 1000000)
        return (value / 1000000) + "m";
    else if(value >= 1000)
        return (value / 1000) + "k";
    else
        return "" + value;
}

public ScreenType getScreenFromString(String value)
{
    foreach(var type in screenTypes)
        if(Enum.GetName(typeof(ScreenType), type).ToLower() == value.ToLower())
            return type;
    return ScreenType.BLANK;
}

public void echoToScreens(List<IMyTextPanel> screens, string text)
{
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

public int getTotalOfItem(MyItemType item)
{
    int tally = 0;
    // Items in monitored storage
    foreach(var box in containers)
        tally += box.GetInventory().GetItemAmount(item).ToIntSafe();
    // Items in input/output of managed assemblers
    foreach(var box in assemblers)
    {
        tally += box.InputInventory.GetItemAmount(item).ToIntSafe();
        tally += box.OutputInventory.GetItemAmount(item).ToIntSafe();
    }
    return tally;
}

public bool isRequestOfItem(MyProductionItem request, MyItemType item)
{
    return request.BlueprintId.ToString().Contains(item.SubtypeId);
}

// Creates a new production request for the given item
public MyDefinitionId createRequest(MyItemType item)
{
    string name = item.SubtypeId;
    if(name == "Explosives")
        name = "ExplosiveComponent";
    else if(name == "Girder")
        name = "GirderComponent";
    else if(name == "Computer")
        name = "ComputerComponent";
    else if(name == "Construction")
        name = "ConstructionComponent";
    else if(name == "Motor")
        name = "MotorComponent";
    return MyDefinitionId.Parse("MyObjectBuilder_BlueprintDefinition/" + name);
}

public void pushRequest(MyItemType item, decimal count)
{
    MyDefinitionId request = createRequest(item);
    
    // Find least-busy viable assembler
    IMyAssembler recipient = null;
    int shortestQueue = INT_MAX;
    List<IMyAssembler> candidates = new List<IMyAssembler>();
    foreach(IMyAssembler ass in assemblers)
        if(ass.IsFunctional && ass.CanUseBlueprint(request))
        {
            int len = 0;
            List<MyProductionItem> queue = new List<MyProductionItem>();
            ass.GetQueue(queue);
            foreach(MyProductionItem recipe in queue)
                len += recipe.Amount.ToIntSafe();
            
            if(len < shortestQueue)
            {
                recipient = ass;
                shortestQueue = len;
            }
        }
    
    if(recipient == null)
        Echo("No viable assembler available for "+item.SubtypeId);
    else
        recipient.AddQueueItem(request, count);
}
