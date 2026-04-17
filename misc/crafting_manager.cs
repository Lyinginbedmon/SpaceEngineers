// == Creation Crafting & Management OS ==

// HOW TO USE
// The default config is automatically loaded into the programmable block's CustomData if it is blank when compiled
// The program will automatically shut off in this event to give you a moment to set it up first
// Adjust the config however you like
// * Prefix - Used to indicate blocks being manipulated by this system
// * CraftRate - How frequently the system updates its crafting requests (higher = less frequently), set to 0 to disable autocrafting entirely
// * CraftStyle - How the system manages the assemblers. Stacked = assembler with shortest queue, Balanced = distributed across all assemblers
// * StorageFlag - The value in a block's CustomData that flags it as monitored storage. Blocks can have multiple flags
// * CrafterFlag - The value in an assembler's CustomData that flags it as useable crafting
// * DisplayUnmanaged - If set to 1, the system will still display items whose thresholds are set to 0
// * Item groups - All sections after [general] are item groups and their corresponding thresholds, refer to the default config for formatting
// Once the config has been adjusted, recompile the program. A spinning icon should appear in the console to indicate activity
//
// ITEM FORMATTING
// Items need to be defined in the config in a particular way to properly match game values
// To that end, they are referred to with a lowercase type then their proper name, separated by a semicolon (:)
// The supported types are: ore, ingot, component, tool, and ammo
// It's important to note that whilst MOST items have a proper name reflecting their translated English name without spaces, not all do!
// For instance, Construction Components are simply Construction in their proper name.
// In most cases, when an item's translated English name includes its type, the type is removed. Such as ore:Iron for Iron Ore.
//
// SETTING UP THE NETWORK
// Go to each cargo container you want to be included, and put the StorageFlag value in its CustomData (main_hold by default)
// Go to each assembler you want to be included, and put the CrafterFlag value in its CustomData (auto_assembler by default)
// Then recompile the programmable block to apply the updates
// For display screens, set their CustomData to any configured item group name (case sensitive)
// 
// CONSOLE COMMANDS
// * clear_queues - Clears the crafting queues of all managed assemblers
// * reset_config - Reverts the block's CustomData to the default config setup, does NOT reload the config
// * reload_config - Loads the config from the block's CustomData without recompiling
// * scan_assemblers - Rescans the local grid for any nominated assemblers
// * scan_displays - Rescans the local grid for any nominated display screens
// * scan_storage - Rescans the local grid for any cargo containers or other inventories of note
// * stop - Immediately halts the program, recompile to restart it
// * stop_crafting - Disables autocrafting

const String version = "4.3";
const int INT_MAX = 2147483647;
const String spinning = "-\\|/";
String config_default = "";

String prefix;
String storageFlag;
String craftFlag;
int craftingRate;
CraftStyle craftingStyle;
bool showUnmanaged;
// All LCD panels
Dictionary<string, List<IMyTextPanel>> screenMap = new Dictionary<string, List<IMyTextPanel>>();
// All cargo containers
List<IMyTerminalBlock> containers = new List<IMyTerminalBlock>();
// All assemblers
List<IMyAssembler> assemblers = new List<IMyAssembler>();

// Definitions of all supported types of cargo container
public enum ContainerType
{
    SMALL,
    MEDIUM,
    MODULAR,
    LARGE
}
public static ContainerType[] containerTypes = new ContainerType[]{ContainerType.SMALL, ContainerType.MEDIUM, ContainerType.MODULAR, ContainerType.LARGE};

// Definitions for different assembler management approaches
public enum CraftStyle
{
    STACKED,
    BALANCED
}
public static CraftStyle[] craftStyles = new CraftStyle[]{CraftStyle.STACKED, CraftStyle.BALANCED};

bool hasClearedScreen = false;
int ticksRunning = 0;

// Library of item groups and their corresponding thresholds
private Dictionary<string, Dictionary<MyItemType, int>> library = new Dictionary<string, Dictionary<MyItemType, int>>();
// Dictionary of monitored components to their configured crafting thresholds
private static Dictionary<MyItemType,int> componentThresholds = new Dictionary<MyItemType, int>();
// Dictionary of monitored components to how many are needed to meet thresholds
private static Dictionary<MyItemType,int> componentRequests = new Dictionary<MyItemType, int>();
private List<string> latestReport = new List<string>();

private static MyIni config = new MyIni();

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    createDefaultConfig();
    
    start();
}

public void start()
{
    // Define component thresholds
    loadConfig(Me.CustomData);
    
    // Identify display screens
    collectScreens();
    
    // Identify monitored cargo inventories
    collectContainers();
    
    // Identify nominated assemblers
    collectCrafters();
}

// Initialises the default config settings
private void createDefaultConfig()
{
    MyIni defConfig = new MyIni();
    
    defConfig.AddSection("general");
    defConfig.Set("general", "Prefix","CRT");
    defConfig.Set("general", "CraftRate", 0);
    defConfig.Set("general", "CraftStyle", "stacked");
    defConfig.Set("general", "StorageFlag", "main_hold");
    defConfig.Set("general", "CrafterFlag", "auto_assembler");
    defConfig.Set("general", "DisplayUnmanaged", true);
    
    defConfig.AddSection("Components A");
    defConfig.Set("Components A", "component:SteelPlate", 100000);
    defConfig.Set("Components A", "component:InteriorPlate", 100000);
    defConfig.Set("Components A", "component:Construction", 50000); 
    defConfig.Set("Components A", "component:SmallTube", 10000); 
    defConfig.Set("Components A", "component:LargeTube", 10000); 
    defConfig.Set("Components A", "component:Girder", 10000); 
    
    defConfig.AddSection("Components B");
    defConfig.Set("Components B", "component:MetalGrid", 1000);
    defConfig.Set("Components B", "component:SolarCell", 1000);
    defConfig.Set("Components B", "component:Display", 1000);
    defConfig.Set("Components B", "component:PowerCell", 1000);
    defConfig.Set("Components B", "component:Motor", 1000);
    defConfig.Set("Components B", "component:Computer", 1000);
    
    defConfig.AddSection("Prototech Components");
    defConfig.Set("Prototech Components", "component:PrototechCapacitor", 100);
    defConfig.Set("Prototech Components", "component:PrototechCircuitry", 100);
    defConfig.Set("Prototech Components", "component:PrototechCoolingUnit", 100);
    defConfig.Set("Prototech Components", "component:PrototechFrame", 20);
    defConfig.Set("Prototech Components", "component:PrototechMachinery", 100);
    defConfig.Set("Prototech Components", "component:PrototechPanel", 200);
    defConfig.Set("Prototech Components", "component:PrototechPropulsionUnit", 100);
    defConfig.Set("Prototech Components", "component:PrototechScrap", 0);
    
    config_default = defConfig.ToString();
}

public void loadConfig(String customData)
{
    // If no config is present, load the default
    if(customData.Length == 0)
    {
        Runtime.UpdateFrequency = UpdateFrequency.Once;
        Me.CustomData = config_default;
        throw new Exception("Blank config detected, resetting");
    }
    
    MyIniParseResult result;
    if(!config.TryParse(customData, out result))
    {
        Runtime.UpdateFrequency = UpdateFrequency.Once;
        throw new Exception(result.ToString());
    }
    
    // Identify system variables
    prefix = config.Get("general", "Prefix").ToString("CRT");
    craftingRate = config.Get("general", "CraftRate").ToInt32(10);
    storageFlag = config.Get("general", "StorageFlag").ToString("main_hold").ToLower();
    craftFlag = config.Get("general", "CrafterFlag").ToString("auto_assembler").ToLower();
    craftingStyle = stringToStyle(config.Get("general", "CraftStyle").ToString("stacked"));
    showUnmanaged = config.Get("general", "ShowUnmanaged").ToBoolean(true);
    
    Me.CustomName = prefix+" System Control";
    
    // Identify all managed item groups
    library.Clear();
    List<string> sections = new List<string>();
    config.GetSections(sections);
    foreach(string section in sections)
    {
        // Skip [general] so we don't set our own mandatory section as a monitored grouping
        if(section == "general")
            continue;
        
        // Identify each item and its corresponding threshold in this grouping
        Dictionary<MyItemType,int> group = new Dictionary<MyItemType,int>();
        List<MyIniKey> keys = new List<MyIniKey>();
        config.GetKeys(section, keys);
        foreach(MyIniKey key in keys)
        {
            string entry = key.Name;
            Nullable<MyItemType> type = stringToItem(entry);
            if(type.HasValue)
                group[type.Value] = config.Get(section, entry).ToInt32(0);
        }
        
        library[section] = group;
    }
}

public void Main(string argument, UpdateType updateSource)
{
    ++ticksRunning;
    componentRequests.Clear();
    
    // Display basic information
    Echo("Creation Crafting & Management "+version+" "+getSpinning());
    Echo("> "+library.Count+" configured item groups");
    foreach(string group in library.Keys)
        Echo("    "+group);
    Echo("> "+containers.Count+" monitored inventories");
    Echo("   Storage Flag: "+storageFlag);
    Echo("> "+assemblers.Count+" managed assemblers");
    Echo("   Crafter Flag: "+craftFlag);
    
    // Update display screens
    foreach(string set in screenMap.Keys)
        handleInvScreens(screenMap[set], library[set], set);
    
    // Update autocrafting handling if enabled
    if(containers.Count == 0)
        Echo("   No inventories to monitor");
    else if(craftingRate <= 0)
        Echo("   Autocrafting protocol disabled");
    else if(assemblers.Count == 0)
        Echo("   No assemblers nominated for autocrafting usage");
    else
    {
        Echo("   Crafting style: "+Enum.GetName(typeof(CraftStyle), craftingStyle));
        if(ticksRunning%craftingRate == 0)
            updateAutoCrafting();
        foreach(string line in latestReport)
            Echo(line);
    }
    
    if(argument.Length > 0)
    {
        string command = argument.ToLower();
        if(command == "reset_config")
            Me.CustomData = config_default;
        else if(command == "reload_config")
            start();
        else if(command == "scan_storage")
            collectContainers();
        else if(command == "scan_assemblers")
            collectCrafters();
        else if(command == "scan_displays")
            collectScreens();
        else if(command == "clear_queues")
        {
            foreach(IMyAssembler ass in assemblers)
                ass.ClearQueue();
        }
        else if(command == "stop")
        {
            Runtime.UpdateFrequency = UpdateFrequency.Once;
            throw new Exception("Script stopped by command");
        }
        else if(command == "stop_crafting")
            craftingRate = 0;
    }
}

// #### MANAGEMENT FUNCTIONS ####

public void handleInvScreens(List<IMyTextPanel> screens, Dictionary<MyItemType,int> itemGroup, string groupName)
{
    hasClearedScreen = false;
    echoToScreens(screens, "= "+groupName+":");
    String entry = "".PadRight(20);
    foreach(MyItemType item in itemGroup.Keys)
    {
        echoToScreens(screens, " > "+item.SubtypeId + ":");
        int tally = getTotalOfItem(item);
        int threshold = itemGroup[item];
        if(threshold <= 0 && showUnmanaged)
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
    latestReport.Clear();
    
    // Dictionary of items that need to be queued up for crafting
    Dictionary<MyItemType,int> craftingQueue = new Dictionary<MyItemType,int>();

    // Calculate excess crafting needs, by subtracting demand from existing queues
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
    
    if(craftingQueue.Count == 0)
    {
        latestReport.Add("Inventory nominal, no crafting required");
        return;
    }
    
    // If any crafting needs to be done, collect all the functioning assemblers
    List<IMyAssembler> crafters = new List<IMyAssembler>();
    foreach(IMyAssembler ass in assemblers)
        if(ass.Enabled && ass.IsFunctional)
            crafters.Add(ass);
    
    latestReport.Add("Latest autocrafting report:");
    foreach(MyItemType item in craftingQueue.Keys)
    {
        latestReport.Add("> "+craftingQueue[item]+"x "+item.SubtypeId);
        pushRequest(item, craftingQueue[item], crafters);
    }
}

// #### UTILITY FUNCTIONS ####

// Returns the spinning progress icon
public char getSpinning()
{
    return spinning[this.ticksRunning % spinning.Length];
}

// Adds the given string to all given screens
public void echoToScreens(List<IMyTextPanel> screens, string text)
{
    addStringToDisplays(screens, text, hasClearedScreen);
    hasClearedScreen = true;
}

// Tallies the given item across all monitored inventories
public int getTotalOfItem(MyItemType item)
{
    int tally = 0;
    // Items in monitored storage
    foreach(var box in containers)
        tally += getItemAmount(box.GetInventory(), item);
    // Items in input/output of managed assemblers
    foreach(var box in assemblers)
        tally += getItemAmount(box.InputInventory, item) + getItemAmount(box.OutputInventory, item);
    return tally;
}

// Provides the given item to the assemblers according to the defined crafting style
public void pushRequest(MyItemType item, decimal count, List<IMyAssembler> crafters)
{
    if(count <= 0)
        return;
    
    MyDefinitionId request = createRequest(item);
    
    // Find all operational assemblers that can craft the item
    List<IMyAssembler> set = new List<IMyAssembler>();
    foreach(IMyAssembler ass in crafters)
        if(ass.CanUseBlueprint(request))
        {
            List<MyProductionItem> queue = new List<MyProductionItem>();
            ass.GetQueue(queue);
            if(queue.Count >= 50)
                continue;
            set.Add(ass);
        }
    if(set.Count == 0)
    {
        latestReport.Add("# No viable assembler available for "+item.SubtypeId);
        return;
    }
    
    // Stacked: Place the full order in the assembler with the shortest queue to reduce power usage
    if(craftingStyle == CraftStyle.STACKED || count == 1)
    {
        IMyAssembler recipient = null;
        int shortestQueue = INT_MAX;
        List<IMyAssembler> candidates = new List<IMyAssembler>();
        foreach(IMyAssembler ass in set)
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
        
        recipient.AddQueueItem(request, count);
    }
    // Balanced: Spread the order across all available assemblers to minimise crafting time
    else if(craftingStyle == CraftStyle.BALANCED)
    {
        int amount = (int)count;
        int perAss = (int)MathHelper.Max(1, MathHelper.CeilToInt(amount / set.Count));
        foreach(IMyAssembler ass in set)
        {
            decimal slice = (decimal)MathHelper.Min(perAss, amount);
            ass.AddQueueItem(request, slice);
            amount -= (int)slice;
            if(amount <= 0)
                break;
        }
    }
    else
        throw new Exception("Selected crafting style unrecognised! Try BALANCED or STACKED");
}

// Collects all LCD panels with an item group name as their CustomData
public void collectScreens()
{
    screenMap.Clear();
    foreach(string type in library.Keys)
        screenMap[type] = new List<IMyTextPanel>();
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid && library.ContainsKey(panel.CustomData))
        {
            string type = panel.CustomData;
            panel.CustomName = prefix+" "+type+" Screen "+screenMap[type].Count;
            panel.ContentType = ContentType.TEXT_AND_IMAGE;
            panel.ShowInTerminal = false;
            panel.ShowInToolbarConfig = false;
            screenMap[type].Add(panel);
        }
}

// Collects all nominated cargo containers, as well as all cargo terminals and connectors, on this grid
public void collectContainers()
{
    containers.Clear();
    List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(cargo);
    Dictionary<ContainerType, List<IMyCargoContainer>> renamedMap = new Dictionary<ContainerType, List<IMyCargoContainer>>();
    foreach(var box in cargo)
        if(box.CubeGrid == Me.CubeGrid)
        {
            string type = box.BlockDefinition.SubtypeName;
            if(isBlockFlagged(box, storageFlag))
            {
                // If this system has primary control of this container, rename it accordingly
                if(isBlockFlaggedPrimary(box, storageFlag) && type.Contains("Container"))
                {
                    ContainerType boxType = containerToType(box);
                    List<IMyCargoContainer> set = renamedMap.ContainsKey(boxType) ? renamedMap[boxType] : new List<IMyCargoContainer>();
                    set.Add(box);
                    renamedMap[boxType] = set;
                }
                containers.Add(box);
            }
            // Add all grid-connected cargo terminals to the monitoring list as writ
            else if(type.Contains("CargoTerminal"))
                containers.Add(box);
            
            // I honestly don't know why cargo containers even have these settings
            box.ShowInTerminal = false;
            box.ShowInToolbarConfig = false;
        }
    
    renameContainers(renamedMap, prefix);
    
    // Add all grid-connected connectors to the monitoring list as writ
    List<IMyShipConnector> connectors = new List<IMyShipConnector>();
    GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
    foreach(var box in connectors)
        if(box.CubeGrid == Me.CubeGrid)
            containers.Add(box);
}

// Collects all nominated assemblers on this grid
public void collectCrafters()
{
    assemblers.Clear();
    List<IMyAssembler> crafters = new List<IMyAssembler>();
    GridTerminalSystem.GetBlocksOfType<IMyAssembler>(crafters);
    int boxes = 0;
    foreach(var ass in crafters)
        if(ass.CubeGrid == Me.CubeGrid && isBlockFlagged(ass, craftFlag))
        {
            string type = "Assembler";
            string subtype = ass.BlockDefinition.SubtypeName;
            
            // Never use survival kits for autocrafting
            if(subtype.Contains("Survival"))
                continue;
            // If this system has primary control of this assembler, rename it accordingly
            else if(isBlockFlaggedPrimary(ass, craftFlag))
            {
                if(subtype.Contains("Prototech"))
                    type = "Prototech Assembler";
                else if(subtype.Contains("Basic"))
                    type = "Basic Assembler";
                ass.CustomName = prefix+" Managed "+type+" "+(++boxes);
            }
            
            ass.ShowInTerminal = false;
            ass.ShowInToolbarConfig = false;
            assemblers.Add(ass);
        }
}

// Assigns a unique name to all monitored containers
public static void renameContainers(Dictionary<ContainerType, List<IMyCargoContainer>> map, string prefix)
{
    foreach(ContainerType type in map.Keys)
    {
        List<IMyCargoContainer> set = map[type];
        string size = Enum.GetName(typeof(ContainerType), type).ToLower();
        size = Char.ToUpperInvariant(size[0]) + size.Substring(1,size.Length-1);
        
        int index = 0;
        foreach(IMyCargoContainer box in set)
            box.CustomName = prefix+" "+size+" Cargo Hold "+(++index);
    }
}

// Converts the given cargo container to its corresponding ContainerType
public static ContainerType containerToType(IMyCargoContainer box)
{
    string type = box.BlockDefinition.SubtypeName;
    if(type.Contains("Medium"))
        return ContainerType.MEDIUM;
    else if(type.Contains("Modular"))
        return ContainerType.MODULAR;
    else if(type.Replace("LargeBlock","").Contains("Large"))
        return ContainerType.LARGE;
    else
        return ContainerType.SMALL;
}

// Converts the given string to a CraftStyle, or STACKED if none matches
public static CraftStyle stringToStyle(string nameIn)
{
    foreach(CraftStyle style in craftStyles)
        if(nameIn.ToLower() == Enum.GetName(typeof(CraftStyle), style).ToLower())
            return style;
    return CraftStyle.STACKED;
}

// Reduces a numerical value to a truncated version, such as 1k or 3m
public static string abbreviateValue(int value)
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

// Adds the given text to every LCD in the list, appending or clearing as appropriate
public static void addStringToDisplays(List<IMyTextPanel> displays, string text, bool append)
{
    foreach(var screen in displays)
        screen.WriteText((append ? "\n" : "") + text, append);
}

// Manually compares items by ID, because Prototech Scrap gets overlooked otherwise for some reason
public static int getItemAmount(IMyInventory inv, MyItemType item)
{
    int tally = 0;
    int index = 0;
    while(index < inv.ItemCount)
    {
        MyInventoryItem? slot = inv.GetItemAt(index);
        MyInventoryItem contents = slot.Value;
        
        if(contents.Type.SubtypeId.Contains(item.SubtypeId))
            tally += contents.Amount.ToIntSafe();
        index++;
    }
    return tally;
}

// Returns true if the given request is of the given item
public static bool isRequestOfItem(MyProductionItem request, MyItemType item)
{
    return request.BlueprintId.ToString().Contains(item.SubtypeId);
}

// Creates a new production request for the given item
public static MyDefinitionId createRequest(MyItemType item)
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
    else if(name == "Thrust")
        name = "ThrustComponent";
    return MyDefinitionId.Parse("MyObjectBuilder_BlueprintDefinition/" + name);
}

// Converts a config item entry into a corresponding item type, if possible
public static Nullable<MyItemType> stringToItem(string nameIn)
{
    String[] elements = nameIn.Split(':');
    if(elements.Length != 2)
        throw new Exception("Item not specified properly: "+nameIn+", must be in format ore:Iron or component:Construction etc");
    String type = elements[0].ToLower();
    String name = elements[1];
    if(type.Equals("ore"))
        return MyItemType.MakeOre(name);
    else if(type.Equals("ingot"))
        return MyItemType.MakeIngot(name);
    else if(type == "component")
        return MyItemType.MakeComponent(name);
    else if(type == "tool")
        return MyItemType.MakeTool(name);
    else if(type == "ammo")
        return MyItemType.MakeAmmo(name);
    
    throw new Exception("Item type unrecognised: "+type+" in "+nameIn);
}

// Returns true if any line of the given block's CustomData matches the given flag string
public static bool isBlockFlagged(IMyTerminalBlock block, string flagIn)
{
    string flags = block.CustomData.ToLower();
    string target = flagIn.ToLower();
    
    // Single line
    if(flags == target)
        return true;
    
    // Multi-line
    foreach(string f in flags.Split('\n'))
        if(f == target)
            return true;
    
    return false;
}

// Returns true if this block has the given flag AND it is the first/only flag it has
public static bool isBlockFlaggedPrimary(IMyTerminalBlock block, string flagIn)
{
    string flags = block.CustomData.ToLower();
    string target = flagIn.ToLower();
    return flags == target || flags.Split('\n')[0] == target;
}
