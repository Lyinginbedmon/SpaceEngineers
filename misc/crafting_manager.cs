// == Creation Crafting & Management OS ==

// HOW TO USE
// The default config is automatically loaded into the programmable block's CustomData if it is blank when compiled
// The program will automatically shut off in this event to give you a moment to set it up first
// Adjust the config however you like
// * Prefix - Used to indicate blocks being manipulated by this system
// * CraftRate - How frequently the system updates its crafting requests (higher = less frequently), set to 0 to disable autocrafting entirely
// * StorageFlag - The value in a block's CustomData that flags it as monitored storage
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
// * reset_config - Reverts the block's CustomData to the default config setup, does NOT reload the config
// * reload_config - Loads the config from the block's CustomData without recompiling
// * scan_assemblers - Rescans the local grid for any nominated assemblers
// * scan_displays - Rescans the local grid for any nominated display screens
// * scan_storage - Rescans the local grid for any cargo containers or other inventories of note
// * stop - Immediately halts the program, recompile to restart it
// * stop_crafting - Disables autocrafting

const String version = "3.0";
const int INT_MAX = 2147483647;
const String spinning = "-\\|/";
const String config_default = 
    "[general]\nPrefix=CRT\nCraftRate=0\nStorageFlag=main_hold\nCrafterFlag=auto_assembler\nDisplayUnmanaged=1\n\n[Components A]\ncomponent:SteelPlate=100000\ncomponent:InteriorPlate=100000\ncomponent:Construction=50000\ncomponent:SmallTube=10000\ncomponent:LargeTube=10000\ncomponent:Girder=10000\n\n[Components B]\ncomponent:MetalGrid=1000\ncomponent:SolarCell=1000\ncomponent:Display=1000\ncomponent:PowerCell=1000\ncomponent:Motor=1000\ncomponent:Computer=1000\n\n[Prototech Components]\ncomponent:PrototechCapacitor=100\ncomponent:PrototechCircuitry=100\ncomponent:PrototechCoolingUnit=100\ncomponent:PrototechFrame=20\ncomponent:PrototechMachinery=100\ncomponent:PrototechPanel=200\ncomponent:PrototechPropulsionUnit=100\ncomponent:PrototechScrap=0";

String prefix;
String storageFlag;
String craftFlag;
int craftingRate;
bool showUnmanaged;
// All LCD panels
Dictionary<string, List<IMyTextPanel>> screenMap = new Dictionary<string, List<IMyTextPanel>>();
// All cargo containers
List<IMyTerminalBlock> containers = new List<IMyTerminalBlock>();
// All assemblers
List<IMyAssembler> assemblers = new List<IMyAssembler>();

bool hasClearedScreen = false;
int ticksRunning = 0;

// Library of item groups and their corresponding thresholds
private Dictionary<string, Dictionary<MyItemType, int>> library = new Dictionary<string, Dictionary<MyItemType, int>>();
// Dictionary of monitored components to their configured crafting thresholds
private static Dictionary<MyItemType,int> componentThresholds = new Dictionary<MyItemType, int>();
// Dictionary of monitored components to how many are needed to meet thresholds
private static Dictionary<MyItemType,int> componentRequests = new Dictionary<MyItemType, int>();

private static MyIni config = new MyIni();

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
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
    
    Me.CustomName = prefix+" System Control";
    
    // Identify system variables
    prefix = config.Get("general", "Prefix").ToString("CRT");
    craftingRate = config.Get("general", "CraftRate").ToInt32(10);
    storageFlag = config.Get("general", "StorageFlag").ToString("main_hold").ToLower();
    craftFlag = config.Get("general", "CrafterFlag").ToString("auto_assembler").ToLower();
    showUnmanaged = config.Get("general", "ShowUnmanaged").ToBoolean(true);
    
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
    Echo("> "+containers.Count+" monitored inventories");
    Echo("> "+assemblers.Count+" accessed assemblers");
    
    // Update display screens
    foreach(string set in screenMap.Keys)
        handleInvScreens(screenMap[set], library[set], set);
    
    // Update autocrafting handling if enabled
    if(containers.Count == 0)
        Echo("No inventories to monitor");
    else if(craftingRate <= 0)
        Echo("Autocrafting protocol disabled");
    else if(assemblers.Count == 0)
        Echo("No assemblers nominated for autocrafting usage");
    else if(ticksRunning%craftingRate == 0)
        updateAutoCrafting();
    
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
    
    Echo("Latest autocrafting report:");
    foreach(MyItemType item in craftingQueue.Keys)
    {
        Echo("> "+craftingQueue[item]+"x "+item.SubtypeId);
        pushRequest(createRequest(item), craftingQueue[item]);
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

// Provides the given item to the nominated assembler with the shortest queue
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
    int boxes = 0;
    foreach(var box in cargo)
        if(box.CubeGrid == Me.CubeGrid)
        {
            string type = box.BlockDefinition.SubtypeName;
            if(box.CustomData.ToLower() == storageFlag)
            {
                if(type.Contains("Container"))
                    renameContainer(box, prefix, ++boxes);
                containers.Add(box);
            }
            else if(type.Contains("CargoTerminal"))
                containers.Add(box);
            
            box.ShowInTerminal = false;
            box.ShowInToolbarConfig = false;
        }
    
    List<IMyShipConnector> connectors = new List<IMyShipConnector>();
    GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
    foreach(var box in connectors)
        if(box.CubeGrid == Me.CubeGrid)
            containers.Add(box);
}

// Renames monitored cargo containers whilst retaining useful identifying information
public static void renameContainer(IMyCargoContainer box, string prefix, int index)
{
    string type = box.BlockDefinition.SubtypeName;
    string size = "Small";
    if(type.Contains("Medium"))
        size = "Medium";
    else if(type.Contains("Modular"))
        size = "Modular";
    else if(type.Replace("LargeBlock","").Contains("Large"))
        size = "Large";
    box.CustomName = prefix+" "+size+" Cargo Hold "+index;
}

// Collects all nominated assemblers on this grid
public void collectCrafters()
{
    assemblers.Clear();
    List<IMyAssembler> crafters = new List<IMyAssembler>();
    GridTerminalSystem.GetBlocksOfType<IMyAssembler>(crafters);
    int boxes = 0;
    foreach(var ass in crafters)
        if(ass.CubeGrid == Me.CubeGrid && ass.CustomData.ToLower() == craftFlag)
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
    return MyDefinitionId.Parse("MyObjectBuilder_BlueprintDefinition/" + name);
}

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
