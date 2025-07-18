/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "cltool.h"
#include <string.h>
#include <chrono>
#include <ctime>
#include "ISDataMappings.h"

using namespace std;

cmd_options_t g_commandLineOptions = {};
serial_port_t g_serialPort;
cInertialSenseDisplay g_inertialSenseDisplay;
static bool g_internal = false;
gpx_flash_cfg_t g_gpxFlashCfg_upload = {};

int cltool_serialPortSendComManager(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend)
{
    (void)cmHandle;
    (void)pHandle;
    return serialPortWrite(&g_serialPort, bufferToSend->buf, bufferToSend->size);
}

bool cltool_setupLogger(InertialSense& inertialSenseInterface)
{   // Enable logging in continuous background mode
    cISLogger::sSaveOptions options;
    options.logType = cISLogger::ParseLogType(g_commandLineOptions.logType);
    options.driveUsageLimitPercent = g_commandLineOptions.logDriveUsageLimitPercent;    // max drive limit in percentage, 0 to disable limit
    options.driveUsageLimitMb = g_commandLineOptions.logDriveUsageLimitMb;              // max drive limit in MB, 0 to disable limit
    options.maxFileSize = g_commandLineOptions.maxLogFileSize;                          // each log file will be no larger than this in bytes
    options.useSubFolderTimestamp = g_commandLineOptions.logSubFolder != cISLogger::g_emptyString;
    options.timeStamp = g_commandLineOptions.logSubFolder;                              // log sub folder name
    return inertialSenseInterface.EnableLogger(
        g_commandLineOptions.enableLogging,
        g_commandLineOptions.logPath,
        options,
        g_commandLineOptions.rmcPreset,
        RMC_OPTIONS_PRESERVE_CTRL);
}

static bool startsWith(const char* str, const char* pre)
{
    size_t lenpre = strlen(pre), lenstr = strlen(str);
    return lenstr < lenpre ? false : strncasecmp(pre, str, lenpre) == 0;
}

static bool matches(const char* str, const char* pre)
{
    size_t lenpre = strlen(pre), lenstr = strlen(str);
    return lenstr != lenpre ? false : strncasecmp(pre, str, lenpre) == 0;
}

#define CL_DEFAULT_BAUD_RATE                        IS_BAUDRATE_DEFAULT
#define CL_DEFAULT_DEVICE_PORT                      "*"
#define CL_DEFAULT_DISPLAY_MODE                     cInertialSenseDisplay::DMODE_SCROLL
#define CL_DEFAULT_LOG_TYPE                         "raw"
#define CL_DEFAULT_LOGS_DIRECTORY                   DEFAULT_LOGS_DIRECTORY
#define CL_DEFAULT_ENABLE_LOGGING                   false
#define CL_DEFAULT_MAX_LOG_FILE_SIZE                1024 * 1024 * 5
#define CL_DEFAULT_LOG_DRIVE_USAGE_LIMIT_PERCENT    0.5f
#define CL_DEFAULT_LOG_DRIVE_USAGE_LIMIT_MB         0
#define CL_DEFAULT_REPLAY_SPEED                     1.0
#define CL_DEFAULT_BOOTLOAD_VERIFY                  false


bool read_get_did_argument(string s, stream_did_t *dataset, std::string &fields)
{
    eDataIDs did = cISDataMappings::Did(s);

    if (did > DID_NULL && did < DID_COUNT)
    {   // DID is valid
        fields = "";
        std::string::size_type pos = s.find('=');
        if (pos != std::string::npos)
        {   // Contains '=' specifying a specific field to get
            fields = s.substr(pos + 1);
            
            // // Validate that the field exists
            // const map_name_to_info_t* map = cISDataMappings::NameToInfoMap(did);
        }

        dataset->did = did;

        return true;
    }

    cout << "Invalid argument: " << s << endl;
    return false;
}

void ConvertIndexedKeysToSequences(YAML::Node& node)
{
    if (!node.IsMap()) return;

    static const std::regex indexedKeyRegex(R"((.+)\[(\d+)\]$)");
    std::vector<std::string> keysToRemove;

    // Copy key-value pairs to avoid iterator invalidation
    std::vector<std::pair<std::string, YAML::Node>> entries;
    for (const auto& it : node)
    {
        const std::string key = it.first.as<std::string>();
        entries.emplace_back(key, it.second);
    }

    for (const auto& [key, value] : entries)
    {
        // Recurse into maps
        if (value.IsMap())
        {
            YAML::Node subNode = value;  // Safe copy
            ConvertIndexedKeysToSequences(subNode);
            node[key] = subNode;  // Assign back
        }

        std::smatch match;
        if (std::regex_match(static_cast<const std::string&>(key), match, indexedKeyRegex))
        {
            const std::string baseKey = match[1];
            const int index = std::stoi(match[2]);

            // Ensure the base key exists and is a sequence
            YAML::Node tmpBase = node[baseKey];
            if (!tmpBase || !tmpBase.IsSequence()) {
                tmpBase = YAML::Node(YAML::NodeType::Sequence);
            }

            // Resize sequence to fit index
            while (tmpBase.size() <= static_cast<size_t>(index)) {
                tmpBase.push_back(YAML::Node());
            }

            tmpBase[index] = value;

            node[baseKey] = tmpBase;  // Write it back
            keysToRemove.push_back(key);
        }
    }

    // Remove indexed keys
    for (const std::string& k : keysToRemove)
    {
        node.remove(k);
    }
}

bool read_get_set_argument(std::string s, YAML::Node &node)
{
    // Preprocess s: wrap with braces if it contains no braces at all
    if (!s.empty() && s.find('{') == std::string::npos && s.find('}') == std::string::npos)
    {
        s = "{" + s + "}";
    }

    // Ensure there is a space after each colon
    for (size_t i = 0; i < s.length(); ++i)
    {
        if (s[i] == ':')
        {
            // If not followed by space and not end of string
            if (i + 1 >= s.length() || s[i + 1] != ' ')
            {
                s.insert(i + 1, " ");
            }
        }
    }

    // Replace each match with quoted version.  This regex matches unquoted keys like: gps1AntOffset[0] or myField[12]
    static const std::regex keyWithIndexRegex(R"((\{[^{}]*?)\b([A-Za-z_][A-Za-z0-9_]*\[\d+\])(?=\s*:))");
    s = std::regex_replace(s, keyWithIndexRegex, "$1\"$2\"");
    
    // Attempt to parse the string as YAML
    try {
        node = YAML::Load(s);
    } catch (const YAML::ParserException& e) {
        std::cerr << "Parser error: " << e.what() << "\n";
        return false;
    } catch (const YAML::Exception& e) {
        std::cerr << "General YAML error: " << e.what() << "\n";
        return false;
    }

    // cout << "Parsed YAML: " << node << endl;
    // Convert indexed keys to sequences i.e. {gps1AntOffset[0]: 1.5, gps1AntOffset[1]: 2.5} becomes {gps1AntOffset: [1.5, 2.5]}
    ConvertIndexedKeysToSequences(node);
    // cout << "Revised YAML: " << node << endl;

    g_commandLineOptions.datasets.clear();
    g_commandLineOptions.outputOnceDid.clear();

    YAML::Node newNode;
    for (const auto& topLevel : node) 
    {
        std::string key = topLevel.first.as<std::string>();
        const YAML::Node& value = topLevel.second;

        stream_did_t dataset = {};
        eDataIDs did = cISDataMappings::Did(key);

        if (did > DID_NULL && did < DID_COUNT)
        {
            dataset.did = did;
            dataset.periodMultiple = 0;

            g_commandLineOptions.datasets.push_back(dataset);
            g_commandLineOptions.outputOnceDid.push_back(dataset.did);

            // Validate fields
            const auto& dataSetMap = *cISDataMappings::NameToInfoMap(did);
            if (value.IsMap())
            {
                for (const auto& field : value)
                {
                    std::string fieldName = field.first.as<std::string>();
                    if (dataSetMap.find(fieldName) == dataSetMap.end())
                    {
                        std::cerr << "Invalid field '" << fieldName << "' in " << cISDataMappings::DataName(did) << "\n";
                        return false;
                    }
                }
            }

            std::string canonicalName = cISDataMappings::DataName(did);
            newNode[canonicalName] = value;
        }
        else
        {
            std::cerr << "Invalid DID name or number: " << key << "\n";
            return false;
        }
    }

    // Update yaml node to ensure keys are DID names instead of DID numbers 
    node = newNode;

    // cout << "Parsed YAML: " << node << endl;

    if (g_commandLineOptions.datasets.empty())
    {
        std::cout << "Invalid argument: " << s << std::endl;
        return false;
    }

    return true;
}

bool read_did_argument(string s, stream_did_t *dataset)
{
    eDataIDs did = cISDataMappings::Did(s);

    if (did > DID_NULL && did < DID_COUNT)
    {   // DID is valid
        dataset->did = did;
        dataset->periodMultiple = cISDataMappings::DefaultPeriodMultiple(did);      // Use default to prevent 1ms period streaming for non-rmc messages

        std::string::size_type pos = s.find('=');
        if (pos != std::string::npos)
        {   // Contains '='
            string m = s.substr(pos + 1);

            if (m.find(" ") == std::string::npos)
            {   // Found period multiple following equal sign (i.e. DID=periodMultiple: 4=4 or DID_INS_1=4)
                dataset->periodMultiple = strtol(m.c_str(), NULL, 10);
            }
        }

        return true;
    }

    return false;
}

string cltool_version()
{
	string info;
#if defined(IS_SDK_DESCRIBE_TAG)
	info += string("") + IS_SDK_DESCRIBE_TAG;
#endif
#if defined(IS_SDK_BUILD_DATE) && defined(IS_SDK_BUILD_TIME)
	info += string(" ") + IS_SDK_BUILD_DATE + " " + IS_SDK_BUILD_TIME;
#endif
	return info;
}

void print_dids()
{
#if defined(INCLUDE_LUNA_DATA_SETS)
    for (eDataIDs id = 0; id < DID_COUNT; id++)
#else
    for (eDataIDs id = 0; id < DID_COUNT; id++)
#endif
    {
        printf("(%d) %s\n", id, cISDataMappings::DataName(id));
    }
    cltool_outputHelp();
}

void enable_display_mode(int mode = cInertialSenseDisplay::DMODE_PRETTY)
{   
    g_commandLineOptions.displayMode = mode;
}

bool cltool_parseCommandLine(int argc, char* argv[])
{
    // defaults
    g_commandLineOptions.baudRate = CL_DEFAULT_BAUD_RATE;
    g_commandLineOptions.comPort = CL_DEFAULT_DEVICE_PORT;
    g_commandLineOptions.displayMode = CL_DEFAULT_DISPLAY_MODE;
    g_commandLineOptions.rmcPreset = 0;
    g_commandLineOptions.enableLogging = CL_DEFAULT_ENABLE_LOGGING;
    g_commandLineOptions.logType = CL_DEFAULT_LOG_TYPE;
    g_commandLineOptions.logPath = CL_DEFAULT_LOGS_DIRECTORY;
    g_commandLineOptions.logSubFolder = cISLogger::CreateCurrentTimestamp();
    g_commandLineOptions.maxLogFileSize = CL_DEFAULT_MAX_LOG_FILE_SIZE;
    g_commandLineOptions.logDriveUsageLimitPercent = CL_DEFAULT_LOG_DRIVE_USAGE_LIMIT_PERCENT;
    g_commandLineOptions.logDriveUsageLimitMb = CL_DEFAULT_LOG_DRIVE_USAGE_LIMIT_MB;
    g_commandLineOptions.replaySpeed = CL_DEFAULT_REPLAY_SPEED;
    g_commandLineOptions.bootloaderVerify = CL_DEFAULT_BOOTLOAD_VERIFY;
    g_commandLineOptions.timeoutFlushLoggerSeconds = 3;
    g_commandLineOptions.nmeaRx = false;
    g_commandLineOptions.nmeaMessage = "";
    g_commandLineOptions.updateBootloaderFilename = "";
    g_commandLineOptions.forceBootloaderUpdate = false;

    g_commandLineOptions.surveyIn.state = 0;
    g_commandLineOptions.surveyIn.maxDurationSec = 15 * 60; // default survey of 15 minutes
    g_commandLineOptions.surveyIn.minAccuracy = 0;
    g_commandLineOptions.outputOnceDid.clear();
    g_commandLineOptions.platformType = -1;
    g_commandLineOptions.updateFirmwareTarget = fwUpdate::TARGET_HOST;
    g_commandLineOptions.runDurationMs = 0; // run until interrupted, by default

    if(argc <= 1)
    {   // Display usage menu if no options are provided
        cltool_outputUsage();
        return false;
    }

    // printf("Arguments: %d\n\n%s\n\n", argc, argv[0]);

    // parse command line.  Keep these options in alphabetic order!
    for (int i = 1; i < argc; i++)
    {
        const char* a = argv[i];
        if (startsWith(a, "--"))
        {   // Treat "--" same as "-"
            a++;
        }

        if (startsWith(a, "-base="))
        {
            g_commandLineOptions.baseConnection = &a[6];
            enable_display_mode();
        }
        else if (startsWith(a, "-baud="))
        {
            g_commandLineOptions.baudRate = strtol(&a[6], NULL, 10);
        }
        else if (startsWith(a, "-chipEraseIMX"))
        {
            g_commandLineOptions.sysCommand = SYS_CMD_MANF_CHIP_ERASE;
        }
        else if (matches(a, "-c") && (i + 1) < argc)
        {
            g_commandLineOptions.comPort = argv[++i];   // use next argument
        }
        else if (startsWith(a, "-dboc"))
        {
            g_commandLineOptions.disableBroadcastsOnClose = true;
        }
        else if (startsWith(a, "-dur="))
        {
            g_commandLineOptions.runDurationMs = (uint32_t)(atof(&a[5])*1000.0);
        }
        else if (startsWith(a, "-dids"))
        {
            print_dids();
            return false;
        }
        else if (startsWith(a, "-did") && (i + 1) < argc)
        {
            while ((i + 1) < argc && !startsWith(argv[i + 1], "-"))    // loop through next arguments that don't start with "-"
            {
                i++;
                stream_did_t dataset = {};
                if (read_did_argument(argv[i], &dataset))    // use next argument
                {
                    if (dataset.periodMultiple == 0)
                    {
                        g_commandLineOptions.outputOnceDid.push_back(dataset.did);
                        g_commandLineOptions.datasets.clear();
                        g_commandLineOptions.displayMode = cInertialSenseDisplay::eDisplayMode::DMODE_QUIET;
                    }
                    g_commandLineOptions.datasets.push_back(dataset);
                }
            }
            enable_display_mode();
        }
        else if (startsWith(a, "-edit"))
        {
            stream_did_t dataset = {};
            if (((i + 1) < argc) && read_did_argument(argv[++i], &dataset))    // use next argument
            {
                g_commandLineOptions.datasetEdit = dataset;
            }
            else
            {   // Invalid argument
                print_dids();
                return false;
            }
        }
        else if (startsWith(a, "-evf="))
        {
            g_commandLineOptions.evFCont.sendEVF = true;

            char* token = strtok((char*)&a[5], ",");
            g_commandLineOptions.evFCont.dest = stoi(token);

            if (g_commandLineOptions.evFCont.dest == 0)
                printf("EVF Target: Primary device\n");
            else if (g_commandLineOptions.evFCont.dest == 1)
                printf("EVF Target: device GNSS1 port\n");
            else if (g_commandLineOptions.evFCont.dest == 2)
                printf("EVF Target: device GNSS2 port\n");
            else
            {
                printf("EVF Target: INVALID\n");
                g_commandLineOptions.evFCont.sendEVF = false;
                continue;
            }

            token = strtok(NULL, ",");
            if (token != NULL)
            {
                g_commandLineOptions.evFCont.evFilter.portMask = stoi(token);
                printf("EVF PortMask: 0x%02x\n", g_commandLineOptions.evFCont.evFilter.portMask);
            }
            else
            {
                printf("EVF PortMask: MISSING! See usage!\n");
                g_commandLineOptions.evFCont.sendEVF = false;
                continue;
            }

            token = strtok(NULL, ",");
            if (token != NULL)
            {
                g_commandLineOptions.evFCont.evFilter.eventMask.priorityLevel = stoi(token);
                printf("EVF PriorityLevel: 0x%d\n", g_commandLineOptions.evFCont.evFilter.eventMask.priorityLevel);
            }
            else
            {
                printf("EVF PriorityLevel: MISSING! See usage!\n");
                g_commandLineOptions.evFCont.sendEVF = false;
                continue;
            }

            token = strtok(NULL, ",");
            if (token != NULL)
            {
                g_commandLineOptions.evFCont.evFilter.eventMask.msgTypeIdMask = stoi(token);
                printf("EVF msgTypeIdMask: 0x%08x\n", g_commandLineOptions.evFCont.evFilter.eventMask.msgTypeIdMask);
            }
            else
            {
                printf("EVF msgTypeIdMask: MISSING! See usage!\n");
                g_commandLineOptions.evFCont.sendEVF = false;
                continue;
            }

            printf("EVF Enabled!");

        }
        else if (startsWith(a, "-evmi=") || startsWith(a, "-evmg="))
        {
            printf("Parsing EVM!\n");

            char* token = strtok((char*)&a[6], ",");

            if (token != NULL)
            {
                g_commandLineOptions.evMCont.outDir = token;
                printf("EVM Directory: %s\n", g_commandLineOptions.evMCont.outDir.c_str());
            }
            else
            {
                printf("EVM Directory missing arg example \"evmi=/tmp/,H,0x2002130,0x00002345\"\n");
                continue;
            }

            token = strtok(NULL, ",");

            if (token != NULL)
            {
                g_commandLineOptions.evMCont.hex = (token[0] == 'H') || (token[0] == 'h');
                
                if (g_commandLineOptions.evMCont.hex)
                    printf("EVM HEX mode!\n");
                else
                    printf("EVM INT mode!\n");
            }
            else
            {
                printf("EVM Mode missing arg example \"evmi=/tmp/,H,20021a0,0x00002f45\"\n");
                continue;
            }

            token = strtok(NULL, ",");
            // allow 0x00000001-0x08000000,0x10000000-0x200bff00 0x40000000-0x5fffffff,  
            for (g_commandLineOptions.evMCont.addrCnt = 0; g_commandLineOptions.evMCont.addrCnt < 10 && token != NULL; g_commandLineOptions.evMCont.addrCnt++)
            {
                int temp = 0;
                if (g_commandLineOptions.evMCont.hex)
                    temp = stoi(token, nullptr, 16);
                else
                    g_commandLineOptions.evMCont.Addrs[g_commandLineOptions.evMCont.addrCnt] = stoi(token);
                
                // allow 0x00000001-0x08000000,0x10000000-0x200bff00 0x40000000-0x5fffffff 
                if ((temp > 0 && temp < 0x08000000) || 
                    (temp > 0x10000000 && temp < 0x200bff00) || 
                    (temp > 0x40000000 && temp < 0x5fffffff))
                {
                    g_commandLineOptions.evMCont.Addrs[g_commandLineOptions.evMCont.addrCnt] = temp;
                    printf("Found valid Address: 0x%08x\n", temp);
                }
                else
                {
                    g_commandLineOptions.evMCont.addrCnt--;
                    printf("0x%08x: is not in valid range(allowed:0x00000001-0x08000000,0x10000000-0x200bff00 0x40000000-0x5fffffff)\n", temp);
                }

                token = strtok(NULL, ",");
            }

            if (g_commandLineOptions.evMCont.addrCnt)
            {
                g_commandLineOptions.evMCont.sendEVM = true;
                g_commandLineOptions.evMCont.IMX = startsWith(a, "-evmi=");

                printf("EVM Enabled!");
            }
        }
        else if (startsWith(a, "-evo"))
        {
            if ((i + 3) < argc)
            {
                g_commandLineOptions.evOCont.extractEv = true;

                g_commandLineOptions.evOCont.inFile = argv[++i];
                g_commandLineOptions.evOCont.logType = argv[++i];
                g_commandLineOptions.evOCont.outFile = argv[++i];

                printf("EVO src file: %s\n", g_commandLineOptions.evOCont.inFile.c_str());
                printf("EVO src file type: %s\n", g_commandLineOptions.evOCont.logType.c_str());
                printf("EVO dest file: %s\n", g_commandLineOptions.evOCont.outFile.c_str());
                printf("EVO Enabled!\n");
            }
            else if ((i + 1) < argc)
                printf("EVO destination file: MISSING! See usage!\n"); 
            else if ((i + 2) < argc)
                printf("EVO SRC file type: MISSING! See usage!\n");
            else
                printf("EVO SRC file: MISSING! See usage!\n");

        }
        else if (startsWith(a, "-factoryReset"))
        {
            g_commandLineOptions.sysCommand = SYS_CMD_MANF_FACTORY_RESET;
        }
        else if (startsWith(a, "-fb"))
        {
            g_commandLineOptions.forceBootloaderUpdate = true;
        }
        else if (startsWith(a, "-flashCfg="))
        {
            g_commandLineOptions.imxflashCfgSet = true;
            g_commandLineOptions.imxFlashCfg = &a[10];
            std::replace(g_commandLineOptions.imxFlashCfg.begin(), g_commandLineOptions.imxFlashCfg.end(), '|', ',');     // Replace '|' with ',' to match the expected format
            enable_display_mode(cInertialSenseDisplay::eDisplayMode::DMODE_QUIET);
        }
        else if (startsWith(a, "-imxFlashCfg="))
        {
            g_commandLineOptions.imxflashCfgSet = true;
            g_commandLineOptions.imxFlashCfg = &a[13];
            std::replace(g_commandLineOptions.imxFlashCfg.begin(), g_commandLineOptions.imxFlashCfg.end(), '|', ',');     // Replace '|' with ',' to match the expected format
            enable_display_mode(cInertialSenseDisplay::eDisplayMode::DMODE_QUIET);
        }
        else if (startsWith(a, "-flashCfg") || startsWith(a, "-imxFlashCfg"))
        {
            g_commandLineOptions.imxflashCfgSet = true;
            enable_display_mode(cInertialSenseDisplay::eDisplayMode::DMODE_QUIET);
        }
        else if (startsWith(a, "-gpxFlashCfg="))
        {
            g_commandLineOptions.gpxflashCfgSet = true;
            g_commandLineOptions.gpxFlashCfg = &a[13];
            std::replace(g_commandLineOptions.gpxFlashCfg.begin(), g_commandLineOptions.gpxFlashCfg.end(), '|', ',');     // Replace '|' with ',' to match the expected format
            enable_display_mode(cInertialSenseDisplay::eDisplayMode::DMODE_QUIET);
        }
        else if (startsWith(a, "-gpxFlashCfg"))
        {
            g_commandLineOptions.gpxflashCfgSet = true;
            enable_display_mode(cInertialSenseDisplay::eDisplayMode::DMODE_QUIET);
        }
        else if (startsWith(a, "-get") && (i + 1) < argc && !startsWith(argv[i + 1], "-"))
        {
            if (!read_get_set_argument(argv[++i], g_commandLineOptions.getNode))    // use next argument
            {
                return false;
            }
            enable_display_mode(cInertialSenseDisplay::eDisplayMode::DMODE_QUIET);
        }
        else if (startsWith(a, "-set") && (i + 1) < argc && !startsWith(argv[i + 1], "-"))
        {
            if (!read_get_set_argument(argv[++i], g_commandLineOptions.setNode))    // use next argument
            {
                return false;
            }
            enable_display_mode(cInertialSenseDisplay::eDisplayMode::DMODE_QUIET);
        }
        else if (startsWith(a, "-hi") || startsWith(a, "--hi"))
        {
            g_internal = true;
            cltool_outputUsage();
            return false;
        }
        else if (startsWith(a, "-h") || startsWith(a, "--h") || startsWith(a, "-help") || startsWith(a, "--help"))
        {
            cltool_outputUsage();
            return false;
        }
        else if (startsWith(a, "-list-devices"))
        {
            g_commandLineOptions.list_devices = true;
            g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_QUIET;
        }
        else if (startsWith(a, "-lmf="))
        {
            g_commandLineOptions.maxLogFileSize = (uint32_t)strtoul(&a[5], NULL, 10);
        }
        else if (startsWith(a, "-lmb="))
        {
            g_commandLineOptions.logDriveUsageLimitMb = (float)atof(&a[5]);
        }
        else if (startsWith(a, "-lms="))
        {
            g_commandLineOptions.logDriveUsageLimitPercent = (float)atof(&a[5]);
        }
        else if (startsWith(a, "-lm"))
        {
            g_commandLineOptions.listenMode = true;
            g_commandLineOptions.disableDeviceValidation = true;
            enable_display_mode();
        }
        else if (startsWith(a, "-log-flush-timeout="))
        {
            g_commandLineOptions.timeoutFlushLoggerSeconds = strtoul(&a[19], NULLPTR, 10);
            enable_display_mode();
        }
        else if (startsWith(a, "-lon"))
        {
            g_commandLineOptions.enableLogging = true;
        }
        else if (startsWith(a, "-lp") && (i + 1) < argc)
        {
            g_commandLineOptions.logPath = argv[++i];    // use next argument;
        }
        else if (startsWith(a, "-lts="))
        {
            const char* subFolder = &a[5];
            if (*subFolder == '1' || startsWith(subFolder, "true"))
            {
                g_commandLineOptions.logSubFolder = cISLogger::CreateCurrentTimestamp();
            }
            else if (*subFolder == '\0' || *subFolder == '0' || startsWith(subFolder, "false"))
            {
                g_commandLineOptions.logSubFolder = cISLogger::g_emptyString;
            }
            else
            {
                g_commandLineOptions.logSubFolder = subFolder;
            }
        }
        else if (startsWith(a, "-lt="))
        {
            g_commandLineOptions.logType = &a[4];
        }
        else if (startsWith(a, "-magRecal"))
        {
            g_commandLineOptions.rmcPreset = 0;
            g_commandLineOptions.magRecal = true;
            g_commandLineOptions.magRecalMode = strtol(a + 9, NULL, 10);
            enable_display_mode();
        }
        else if (startsWith(a, "-nmea="))
        {
            g_commandLineOptions.nmeaMessage = &a[6];
            g_commandLineOptions.nmeaRx = true;
        }
        else if (startsWith(a, "-nmea"))
        {
            g_commandLineOptions.nmeaRx = true;
        }
        else if (startsWith(a, "-platform"))
        {
            #define PLATFORM_TYPE_TAG_LEN    10
            if(strlen(a) <= PLATFORM_TYPE_TAG_LEN || !isdigit(a[PLATFORM_TYPE_TAG_LEN]))
            {
                cout << "Platform type not specified.\n\n";
                return false;
            }

            int platformType = (uint32_t)strtoul(&a[PLATFORM_TYPE_TAG_LEN], NULL, 10);
            if (platformType < 0 || platformType >= PLATFORM_CFG_TYPE_COUNT)
            {
                cout << "Invalid platform type: " << platformType << "\n\n";
                return false;
            }
            else
            {   // Valid platform
                g_commandLineOptions.platformType = platformType;
            }
        }
        else if (startsWith(a, "-presetGPXPPD"))
        {
            g_commandLineOptions.rmcPreset = RMC_PRESET_GPX_PPD;
            enable_display_mode();
        }
        else if (startsWith(a, "-presetPPD"))
        {
            g_commandLineOptions.rmcPreset = RMC_PRESET_IMX_PPD_GROUND_VEHICLE;
            enable_display_mode();
        }
        else if (startsWith(a, "-presetINS"))
        {
            g_commandLineOptions.rmcPreset = RMC_PRESET_INS;
            enable_display_mode();
        }
        else if (startsWith(a, "-persistent"))
        {
            g_commandLineOptions.persistentMessages = true;
        }
        else if (startsWith(a, "-q"))
        {
            g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_QUIET;
        }
        else if (startsWith(a, "-raw-out"))
        {
            g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_RAW_PARSE;
        }

        else if (startsWith(a, "-rp") && (i + 1) < argc)
        {
            g_commandLineOptions.replayDataLog = true;
            g_commandLineOptions.logPath = argv[++i];    // use next argument
            enable_display_mode();
        }
        else if (startsWith(a, "-rs="))
        {
            g_commandLineOptions.replayDataLog = true;
            g_commandLineOptions.replaySpeed = (float)atof(&a[4]);
            enable_display_mode();
        }
        else if (startsWith(a, "-reset"))
        {
            g_commandLineOptions.softwareReset = true;
        }		
		else if (startsWith(a, "-romBootloader"))
		{
			g_commandLineOptions.sysCommand = SYS_CMD_MANF_ENABLE_ROM_BOOTLOADER;
		}
		else if (startsWith(a, "-rover="))
		{
			g_commandLineOptions.roverConnection = &a[7];

            // DID_GPS1_POS must be enabled for NTRIP VRS to supply rover position.
            stream_did_t dataset = {};
            read_did_argument("DID_GPS1_POS", &dataset);
            g_commandLineOptions.datasets.push_back(dataset);
            enable_display_mode();
        }
        else if (startsWith(a, "-r"))
        {
            g_commandLineOptions.replayDataLog = true;
            enable_display_mode();
        }
        else if (startsWith(a, "-stats"))
        {
            enable_display_mode(cInertialSenseDisplay::DMODE_STATS);
        }
        else if (startsWith(a, "-survey="))
        {
            g_commandLineOptions.rmcPreset = 0;
            g_commandLineOptions.surveyIn.state = strtol(a + 8, NULL, 10);
            int maxDurationSec = strtol(a + 10, NULL, 10);
            if (maxDurationSec > 5)
            {
                g_commandLineOptions.surveyIn.maxDurationSec = maxDurationSec;
            }
            enable_display_mode();
        }
        else if (startsWith(a, "-sysCmd="))
        {
            g_commandLineOptions.sysCommand = (uint32_t)strtoul(&a[8], NULL, 10);
            switch(g_commandLineOptions.sysCommand)
            {   // Disable device validation
                case SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE:        g_commandLineOptions.disableDeviceValidation = true;    break;
            }
        }
        else if (startsWith(a, "-s"))
        {
            enable_display_mode(cInertialSenseDisplay::DMODE_SCROLL);
        }
        else if (startsWith(a, "-ub") && (i + 1) < argc)
        {
            g_commandLineOptions.updateFirmwareTarget = fwUpdate::TARGET_HOST;      // use legacy firmware update mechanism
            g_commandLineOptions.updateBootloaderFilename = argv[++i];              // use next argument
        }
        else if (startsWith(a, "-uf") && (i + 1) < argc)
        {
            if ((strcmp(a, "-ufpkg") == 0) && (i + 1) < argc)
            {
                g_commandLineOptions.updateFirmwareTarget = fwUpdate::TARGET_GPX1;          // use the new firmware update mechanism and target the GPX specifically
                g_commandLineOptions.fwUpdateCmds.push_back(string("package=") + string(argv[++i]));
                enable_display_mode(cInertialSenseDisplay::DMODE_QUIET);                    // Disable ISDisplay cInertialSenseDisplay output
            }
            else if ((strcmp(a, "-uf-cmd") == 0) && (i + 1) < argc)
            {
                g_commandLineOptions.updateFirmwareTarget = fwUpdate::TARGET_GPX1;          // use the new firmware update mechanism and target the GPX specifically
                splitString(string(argv[++i]), ',', g_commandLineOptions.fwUpdateCmds);
                enable_display_mode(cInertialSenseDisplay::DMODE_QUIET);                    // Disable ISDisplay cInertialSenseDisplay output
            }
            else
            {
                g_commandLineOptions.updateFirmwareTarget = fwUpdate::TARGET_HOST;  // use legacy firmware update mechanism
                g_commandLineOptions.updateAppFirmwareFilename = argv[++i];         // use next argument
            }
        }
        else if (startsWith(a, "-uv"))
        {
            g_commandLineOptions.updateFirmwareTarget = fwUpdate::TARGET_HOST;      // use legacy firmware update mechanism
            g_commandLineOptions.bootloaderVerify = true;
        }
        else if (startsWith(a, "-vd"))
        {
            g_commandLineOptions.disableDeviceValidation = true;
        }
        else if (startsWith(a, "-verbose"))
        {
            g_commandLineOptions.verboseLevel = ISBootloader::IS_LOG_LEVEL_INFO;
            if (a[8] == '=')
                g_commandLineOptions.verboseLevel = atoi(&a[9]);
        }
        else if (startsWith(a, "-v") || startsWith(a, "--version"))
        {
            cout << cltool_version() << endl;
            return false;
        }
        else
        {
            cout << "Unrecognized command line option: " << a << endl;
            cltool_outputHelp();
            return false;
        }
    }

    // We are either using a serial port or replaying data
    if ((g_commandLineOptions.comPort.length() == 0) && !g_commandLineOptions.replayDataLog)
    {
        cltool_outputUsage();
        return false;
    }
    else if (g_commandLineOptions.updateAppFirmwareFilename.length() != 0 && g_commandLineOptions.comPort.length() == 0)
    {
        cout << "Use DEVICE_PORT option \"-c \" with bootloader" << endl;
        return false;
    }
    else if (g_commandLineOptions.updateBootloaderFilename.length() != 0 && g_commandLineOptions.comPort.length() == 0)
    {
        cout << "Use DEVICE_PORT option \"-c \" with bootloader" << endl;
        return false;
    }

    return true;
}

bool cltool_replayDataLog()
{
    if (g_commandLineOptions.logPath.length() == 0)
    {
        cout << "Please specify the replay log path!" << endl;
        return false;
    }

    cISLogger logger;
    if (!logger.LoadFromDirectory(g_commandLineOptions.logPath, cISLogger::ParseLogType(g_commandLineOptions.logType), { "ALL" }))
    {
        cout << "Failed to load log files: " << g_commandLineOptions.logPath << endl;
        return false;
    }

    cout << "Replaying log files: " << g_commandLineOptions.logPath << endl;
    p_data_buf_t *data;
    // for (int d=0; d<logger.DeviceCount(); d++)
    for (auto dl : logger.DeviceLogs())
    {
        if (logger.DeviceCount() > 1)
        {
            printf("Device SN%d: \n", dl->SerialNumber());
        }
        while (((data = logger.ReadData(dl)) != NULL) && !g_inertialSenseDisplay.ExitProgram())
        {
            p_data_t d = {data->hdr, data->buf};
            g_inertialSenseDisplay.ProcessData(&d, g_commandLineOptions.replayDataLog, g_commandLineOptions.replaySpeed);
            g_inertialSenseDisplay.PrintData();
        }
    }

    cout << "Done replaying log files: " << g_commandLineOptions.logPath << endl;
    g_inertialSenseDisplay.Goodbye();
    return true;
}

void event_outputEvToFile(string fileName, uint8_t* data, int len)
{
    std::ofstream outfile;

    outfile.open(fileName, std::ios_base::app | std::ios_base::binary); // append instead of overwrite
    outfile.write((const char*)data, len);
    outfile.close();
}

bool cltool_extractEventData()
{
    is_comm_instance_t c;
    uint8_t evScratch[1028 + DID_EVENT_HEADER_SIZE];
    c.rxBuf.start = evScratch;
    c.rxBuf.size = 1028 + DID_EVENT_HEADER_SIZE;
    
    std::time_t logTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    if (g_commandLineOptions.evOCont.inFile.length() == 0)
    {
        cout << "Please specify the parse log path!" << endl;
        return false;
    }

    if (g_commandLineOptions.evOCont.outFile.length() == 0)
    {
        cout << "Please specify the output log path!" << endl;
        return false;
    }

    cISLogger logger;
    if (!logger.LoadFromDirectory(g_commandLineOptions.evOCont.inFile, cISLogger::ParseLogType(g_commandLineOptions.evOCont.logType), { "ALL" }))
    {
        cout << "Failed to load log files: " << g_commandLineOptions.evOCont.inFile << endl;
        return false;
    }


    // time do handle our output dir
    // does it exist?
    struct stat info;
    if (stat(g_commandLineOptions.evOCont.outFile.c_str(), &info) == 0)
    {
        if (info.st_mode & S_IFDIR)
            cout << "Found output dir: " << g_commandLineOptions.evOCont.outFile << endl;
        else
        {
            cout << "Output dir not a folder: " << g_commandLineOptions.evOCont.outFile << endl;
            return false;
        }
    }
    else
    {
        // the folder does not exist try to create it
        // creating output files
        #if PLATFORM_IS_WINDOWS
            if (mkdir(g_commandLineOptions.evOCont.outFile.c_str()) == 0)
        #else
            if (mkdir(g_commandLineOptions.evOCont.outFile.c_str(), 0777))
        #endif

            cout << "Created output dir: " << g_commandLineOptions.evOCont.outFile << endl;
        else
        {
            cout << "Failed to created output dir: " << g_commandLineOptions.evOCont.outFile << endl;
            return false;
        }
    }

    cout << "Parsing log files: " << g_commandLineOptions.evOCont.inFile << endl;

    p_data_buf_t* data;
    // for (int d=0; d<logger.DeviceCount(); d++)
    for (auto dl : logger.DeviceLogs())
    {

        string deviceFolder = g_commandLineOptions.evOCont.outFile + "/SN-" + std::to_string(dl->SerialNumber());
        
        if (stat(deviceFolder.c_str(), &info) == 0)
        {
            if (!(info.st_mode & S_IFDIR))
            {
                cout << "Output dir not a folder skipping: " << deviceFolder << endl;
                continue;
            }
        }
        else
        {
            // the folder does not exist try to create it
            // creating output files
            #if PLATFORM_IS_WINDOWS
                if (mkdir(deviceFolder.c_str()) == 0)
            #else
                if (mkdir(deviceFolder.c_str(), 0777))
            #endif
                cout << "Created output dir: " << deviceFolder << endl;
            else
            {
                cout << "Failed to created output dir: " << deviceFolder << endl;
                continue;
            }
        }

        int count = 0;

        // cycle through data
        while (((data = logger.ReadData(dl)) != NULL))
        {
            p_data_t d = { data->hdr, data->buf };

            if (d.hdr.id == DID_EVENT)
            {
                did_event_t* ev = (did_event_t*)data->buf;
                memset(evScratch, 0, 1028 + DID_EVENT_HEADER_SIZE);
                memcpy(evScratch, ev->data, ev->length);

                string fileName;

                switch (ev->msgTypeID)
                {
                    case EVENT_MSG_TYPE_ID_RAW:  fileName = deviceFolder + "/out.raw"; break;
                    case EVENT_MSG_TYPE_ID_ASCII: fileName = deviceFolder + "/out.txt";  break;
                    case EVENT_MSG_TYPE_ID_RTMC3_RCVR1: 
                        fileName = deviceFolder + "/rcvr1.rtcm"; 
                        c.rxBuf.size = ev->length;
                        c.rxBuf.head = evScratch;
                        c.rxBuf.end = evScratch + ev->length;
                        c.rxBuf.tail = evScratch + ev->length;
                        c.rxBuf.scan = evScratch;
                        
                        c.processPkt = nullptr;

                        is_comm_parse_timeout(&c, 0);
                        break;
                    case EVENT_MSG_TYPE_ID_RTMC3_RCVR2: fileName = deviceFolder + "/rcvr2.rtcm";  break;
                    case EVENT_MSG_TYPE_ID_RTMC3_EXT: fileName = deviceFolder + "/rcvr_ext.rtcm";  break;
                    case EVENT_MSG_TYPE_ID_SONY_BIN_RCVR1: fileName = deviceFolder + "/rcvr1.sbp";  break;
                    case EVENT_MSG_TYPE_ID_SONY_BIN_RCVR2: fileName = deviceFolder + "/rcvr2.sbp";  break;
                    default: 
                        fileName = deviceFolder + "/UNKNOWN_" + std::to_string(ev->msgTypeID) + ".Bin";
                        printf("Event type %d found but is not supported. Output at: %s\n", ev->msgTypeID, fileName.c_str());
                        break;
                }
                event_outputEvToFile(fileName, evScratch, ev->length);
            }

            if(++count % 5000 == 0)
                printf("Read %d msgs from SN-%d\n", count, dl->SerialNumber());

        }
    }

    cout << "Done parsing log files: " << g_commandLineOptions.evOCont.inFile << endl;
    return true;
}

void cltool_outputUsage()
{
	cout << boldOff;
	cout << "-----------------------------------------------------------------" << endlbOn;
	cout << "CLTool - " << boldOff << cltool_version() << endl;
	cout << endl;
	cout << "    Command line utility for communicating, logging, and updating firmware with Inertial Sense product line." << endl;
	cout << endlbOn;
	cout << "EXAMPLES" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -did DID_INS_1 DID_GPS1_POS DID_PIMU " << EXAMPLE_SPACE_1 << " # stream DID messages" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -did 4 13 3           " << EXAMPLE_SPACE_1 << " # stream same as line above" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -did 3=5              " << EXAMPLE_SPACE_1 << " # stream DID_PIMU at startupNavDtMs x 5" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -presetPPD            " << EXAMPLE_SPACE_1 << " # stream post processing data (PPD) with INS2" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -presetPPD -lon -lts=1" << EXAMPLE_SPACE_1 << " # stream PPD + INS2 data, logging, dir timestamp" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -edit DID_FLASH_CONFIG" << EXAMPLE_SPACE_1 << " # edit DID_FLASH_CONFIG message" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -baud=115200 -did 5 13=10 " << " # stream at 115200 bps, GPS streamed at 10x startupGPSDtMs" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c * -baud=921600              "                    << EXAMPLE_SPACE_2 << " # 921600 bps baudrate on all serial ports" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -rp " <<     EXAMPLE_LOG_DIR                                              << " # replay log files from a folder" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -rover=RTCM3:192.168.1.100:7777:mount:user:password         # Connect to RTK NTRIP base" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -get 1,4,13,DID_GPS1_POS                                    # Return specific DIDs" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -get \"{DID_INS_1: {insStatus, theta}, DID_INS_2: {qn2b}}\"   # Return portion of two DIDs" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -set \"{DID_FLASH_CONFIG: {gps1AntOffset[1]: 0.8}}\"          # Set one value in DID array" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -set \"{DID_FLASH_CONFIG: {gps1AntOffset: [0.8, 0.0, 1.2]}}\" # Set values in DID" << endlbOff;
	cout << endlbOn;
	cout << "EXAMPLES (Firmware Update)" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -ufpkg fw/IS-firmware.fpkg" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -uf " << EXAMPLE_FIRMWARE_FILE << " -ub " << EXAMPLE_BOOTLOADER_FILE << " -uv" << endlbOff;
	cout << endlbOn;
	cout << "OPTIONS (General)" << endl;
	cout << "    -baud=" << boldOff << "BAUDRATE  Set serial port baudrate.  Options: " << IS_BAUDRATE_115200 << ", " << IS_BAUDRATE_230400 << ", " << IS_BAUDRATE_460800 << ", " << IS_BAUDRATE_921600 << " (default)" << endlbOn;
	cout << "    -c " << boldOff << "DEVICE_PORT  Select serial port. Set DEVICE_PORT to \"*\" for all ports or \"*4\" for only first four." << endlbOn;
	cout << "    -dboc" << boldOff << "           Send stop-broadcast command `$STPB` on close." << endlbOn;
	cout << "    -h --help" << boldOff << "       Display this help menu." << endlbOn;
    cout << "    -list-devices" << boldOff << "   Discovers and prints a list of discovered Inertial Sense devices and connected ports." << endlbOn;
    cout << "    -lm" << boldOff << "             Listen mode for ISB. Disables device verification (-vd) and does not send stop-broadcast command on start." << endlbOn;
	cout << "    -magRecal[n]" << boldOff << "    Recalibrate magnetometers: 0=multi-axis, 1=single-axis" << endlbOn;
	cout << "    -nmea=[s]" << boldOff << "       Send NMEA message s with added checksum footer. Display rx messages. (`-nmea=ASCE,0,GxGGA,1`)" << endlbOn;
	cout << "    -nmea" << boldOff << "           Listen mode for NMEA message without sending stop-broadcast command `$STPB` at start." << endlbOn;
	cout << "    -q" << boldOff << "              Quiet mode, no display." << endlbOn;
    cout << "    -raw-out" << boldOff << "        Outputs all data in a human-readable raw format (used for debugging/learning the ISB protocol)." << endlbOn;
	cout << "    -reset         " << boldOff << " Issue software reset." << endlbOn;
	cout << "    -s" << boldOff << "              Scroll displayed messages to show history." << endlbOn;
	cout << "    -stats" << boldOff << "          Display statistics of data received." << endlbOn;
	cout << "    -survey=[s],[d]" << boldOff << " Survey-in and store base position to refLla: s=[" << SURVEY_IN_STATE_START_3D << "=3D, " << SURVEY_IN_STATE_START_FLOAT << "=float, " << SURVEY_IN_STATE_START_FIX << "=fix], d=durationSec" << endlbOn;
	cout << "    -sysCmd=[c]" << boldOff << "     Send DID_SYS_CMD c (see eSystemCommand) command then exit the program." << endlbOn;
    cout << "    -vd" << boldOff << "             Disable device validation.  Use to keep port(s) open even if device response is not received." << endlbOn;
    cout << "    -verbose[=n] " << boldOff << "   Enable verbose event logging. Use optional '=n' to specify log level between 0 (errors only) and 99 (all events)" << endlbOn;
	cout << "    -v" << boldOff << "              Print version information." << endlbOn;

	cout << endlbOn;
	cout << "OPTIONS (Special)" << endl;
	cout << "    -factoryReset " << boldOff << "  Reset IMX flash config to factory defaults." << endlbOn;
	cout << "    -romBootloader " << boldOff << " Reboot into ROM bootloader mode.  Requires power cycle and reloading bootloader and firmware." << endlbOn;
	if (g_internal)
	{
	cout << "    -chipEraseIMX " << boldOff << "  CAUTION!!! Erase everything on IMX (firmware, config, calibration, etc.)" << endlbOn;
	cout << "    -platform=[t]" << boldOff << "   CAUTION!!! Sets the manufacturing platform type in OTP memory (only get 15 writes)." << endlbOn;
	}

	cout << endlbOn;
	cout << "OPTIONS (Event)" << endl;
    cout << "    -evf=[t],[po],[pr],[id]" << boldOff << "    Sets which DID_EVENT's can be broadcast for debug purposes." << endlbOn;
    cout << "         target:" << boldOff << "        t=[0=device, 1=device's GNSS1 port, 2=device's GNSS2 port]," << endlbOn;
    cout << "         portMask:" << boldOff << "      po=[0x80=currentPort, 0x08=USB port, 0x04=UART2, 0x02=UART1, 0x01=UART)]," << endlbOn;
    cout << "         priorityLevel:" << boldOff << " pr=[Priority ID's to be enabled. See:eEventPriority for protocol EV_ID values]." << endlbOn; 
    cout << "         " << boldOff << "    It is recommended to have a minimum level of 1 at all times to allow broadcast of critical errors."  << endlbOn;
    cout << "         msgTypeIdMask:" << boldOff << " id=[Protocol ID's to be enabled. Mask together protocol EV_ID value (0x01 << EV_ID)." << endlbOn;
    cout << "         " << boldOff << "    See:eEventProtocol for protocol EV_ID values]. It is recommended to mask (0x01 << EVENT_MSG_TYPE_ID_ASCII)" << endlbOn;
    cout << "         " << boldOff << "    at all times to allow broadcast of critical errors." << endlbOn;

	cout << endlbOn;
	cout << "OPTIONS (Firmware Update)" << endl;
    cout << "    -ufpkg " << boldOff << "FILEPATH Update firmware using firmware package file (.fpkg) at FILEPATH." << endlbOn;
	cout << "    -uf " << boldOff << "FILEPATH    Update app firmware using .hex file FILEPATH.  Add -baud=115200 for systems w/ baud limits." << endlbOn;
	cout << "    -ub " << boldOff << "FILEPATH    Update bootloader using .bin file FILEPATH if version is old. Must be used with option -uf." << endlbOn;
	cout << "    -fb " << boldOff << "            Force bootloader update regardless of the version." << endlbOn;
    cout << "    -uv " << boldOff << "            Run verification after application firmware update." << endlbOn;

	cout << endlbOn;
	cout << "OPTIONS (Messages)" << endl;
    cout << "    -get <DID1>,<DID2>,...                  " << boldOff << " Return values of dataset(s). DID may be a name or number." << endlbOn;
    cout << "    -get \"{<DID>: {<FIELD1>,<FIELD2>,...}}\" " << boldOff << " Return values of dataset(s). DID may be a name or number. YAML input format." << endlbOn;
    cout << "                                            " << boldOff << " Examples: -get 1,4,12,DID_GPS1_POS" << endlbOn;
    cout << "                                            " << boldOff << "           -get \"{DID_INS_1,DID_GPS1_POS}\"" << endlbOn;
    cout << "                                            " << boldOff << "           -get \"{DID_INS_1: {insStatus, theta}, DID_INS_2: {qn2b}}\"" << endlbOn;
    cout << "    -set \"{<DID>: {<FIELD1>: <VALUE>, ...}}\"" << boldOff << " Set values of dataset(s). DID may be a number or name. YAML input format." << endlbOn;
    cout << "                                            " << boldOff << " Examples: -set \"{DID_FLASH_CONFIG: {gps1AntOffset: [0.8, 0.0, 1.2]}}\"" << endlbOn;
    cout << "                                            " << boldOff << "           -set \"{DID_FLASH_CONFIG: {gps1AntOffset[2]: 1.2}}\"" << endlbOn;
    cout << "                                            " << boldOff << "           -set \"{12: {ioConfig: 0x1a2b012c, ser2BaudRate: 921600}}\"" << endlbOn;
	cout << "    -did [DID#<=PERIODMULT> DID#<=PERIODMULT> ...]" << boldOff << "  Stream 1 or more datasets and display w/ compact view." << endlbOn;
	cout << "    -edit [DID#<=PERIODMULT>]                     " << boldOff << "  Stream and edit 1 dataset." << endlbOff;
	cout << "          Each DID# can be the DID number or name and appended with <=PERIODMULT> to decrease message frequency. " << endlbOff;
	cout << "          Message period = source period x PERIODMULT. PERIODMULT is 1 if not specified." << endlbOff;
	cout << "          Common DIDs: DID_INS_1, DID_INS_2, DID_INS_4, DID_PIMU, DID_IMU, DID_GPS1_POS," << endlbOff;
	cout << "          DID_GPS2_RTK_CMP_REL, DID_BAROMETER, DID_MAGNETOMETER, DID_FLASH_CONFIG (see data_sets.h for complete list)" << endlbOn;
	cout << "    -dids          " << boldOff << " Print list of all DID datasets" << endlbOn;
	cout << "    -persistent    " << boldOff << " Save current streams as persistent messages enabled on startup" << endlbOn;
	cout << "    -presetPPD     " << boldOff << " Send RMC preset to enable IMX post processing data (PPD) stream" << endlbOn;
	cout << "    -presetINS     " << boldOff << " Send RMC preset to enable INS data stream" << endlbOn;
	cout << "    -presetGPXPPD  " << boldOff << " Send RMC preset to enable GPX post processing data (PPD) stream" << endlbOn;
	cout << endlbOn;
	cout << "OPTIONS (Logging to file, disabled by default)" << endl;
	cout << "    -lon" << boldOff << "            Enable logging" << endlbOn;
	cout << "    -lt=" << boldOff << "TYPE        Log type: raw (default), dat, sdat, kml or csv" << endlbOn;
	cout << "    -lp " << boldOff << "PATH        Log data to path (default: ./" << CL_DEFAULT_LOGS_DIRECTORY << ")" << endlbOn;
	cout << "    -lmb=" << boldOff << "MB         File culling: Log drive usage limit in MB. (default: " << CL_DEFAULT_LOG_DRIVE_USAGE_LIMIT_MB << "). `-lmb=0 -lms=0` disables file culling." << endlbOn;
	cout << "    -lms=" << boldOff << "PERCENT    File culling: Log drive space limit in percent of total drive, 0.0 to 1.0. (default: " << CL_DEFAULT_LOG_DRIVE_USAGE_LIMIT_PERCENT << ")" << endlbOn;
	cout << "    -lmf=" << boldOff << "BYTES      Log max file size in bytes (default: " << CL_DEFAULT_MAX_LOG_FILE_SIZE << ")" << endlbOn;
	cout << "    -lts=" << boldOff << "0          Log sub folder, 0 or blank for none, 1 for timestamp, else use as is" << endlbOn;
	cout << "    -r" << boldOff << "              Replay data log from default path" << endlbOn;
	cout << "    -rp " << boldOff << "PATH        Replay data log from PATH" << endlbOn;
	cout << "    -rs=" << boldOff << "SPEED       Replay data log at x SPEED. SPEED=0 runs as fast as possible." << endlbOn;
	cout << endlbOn;
	cout << "OPTIONS (READ flash config) - DEPRECATED, use `-get` instead" << endl;
	cout << "    -imxFlashCfg" << boldOff  <<  "                                # List all \"keys\" and \"values\" in IMX" << endlbOn;
	cout << "    -gpxFlashCfg" << boldOff  <<  "                                # List all \"keys\" and \"values\" in GPX" << endlbOn;
	cout << "    \"-imxFlashCfg=[key]|[key]|[key]\"" << boldOff << "            # List specific IMX values" <<  endlbOn;
	cout << "    \"-gpxFlashCfg=[key]|[key]|[key]\"" << boldOff << "            # List specific GPX values" <<  endlbOn;
	cout << endl;
	cout << "OPTIONS (WRITE flash config) - DEPRECATED, use `-set` instead" << endl;
	cout << "    \"-imxFlashCfg=[key]=[value]|[key]=[value]\"" << boldOff << "  # Set key / value pairs in IMX flash config. " << endlbOn;
	cout << "    \"-gpxFlashCfg=[key]=[value]|[key]=[value]\"" << boldOff << "  # Set key / value pairs in GPX flash config. " << endlbOn;
	cout << "        " << boldOff <<   "                                        # Surround with \"quotes\" when using pipe operator." << endlbOn;
	cout << "EXAMPLES" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c " << EXAMPLE_PORT << " -imxFlashCfg  " << boldOff << "# Read from device and print all keys and values" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c " << EXAMPLE_PORT << " \"-imxFlashCfg=insOffset[1]=1.2|ser2BaudRate=115200\"  " << boldOff << "# Set multiple values" << endlbOn;
	cout << endl;
	cout << "OPTIONS (RTK Rover / Base)" << endlbOn;
	cout << "    -rover=" << boldOff << "[type]:[IP or URL]:[port]:[mountpoint]:[username]:[password]" << endl;
	cout << "        As a rover (client), receive RTK corrections.  Examples:" << endl;
	cout << "            -rover=TCP:RTCM3:192.168.1.100:7777:mountpoint:username:password   (NTRIP)" << endl;
	cout << "            -rover=TCP:RTCM3:192.168.1.100:7777" << endl;
	cout << "            -rover=TCP:UBLOX:192.168.1.100:7777" << endl;
	cout << "            -rover=SERIAL:RTCM3:" << EXAMPLE_PORT << ":57600        (port, baud rate)" << endlbOn;
	cout << "    -base=" << boldOff << "[IP]:[port]   As a Base (sever), send RTK corrections.  Examples:" << endl;
	cout << "            -base=TCP::7777                             (IP is optional)" << endl;
	cout << "            -base=TCP:192.168.1.43:7777" << endl;
	cout << "            -base=SERIAL:" << EXAMPLE_PORT << ":921600" << endl;
	cout << endlbOn;	
	cout << "CLTool - " << boldOff << cltool_version() << endl;

    cout << boldOff;   // Last line.  Leave bold text off on exit.
}

void cltool_outputHelp()
{
    cout << endlbOff << "Run \"" << boldOn << "cltool -h" << boldOff << "\" to display the help menu." << endl;
}

bool cltool_updateImxFlashCfg(InertialSense& inertialSenseInterface, string flashCfgString)
{
    if (!inertialSenseInterface.WaitForImxFlashCfgSynced())
    {
        cout << "Failed to sync IMX flash config." << endl;
        return false;
    }
    nvm_flash_cfg_t imxFlashCfg;
    inertialSenseInterface.ImxFlashConfig(imxFlashCfg);

    if (flashCfgString.find('=') != std::string::npos)
    {   // Write to flash config
        if (!cISDataMappings::StringToDidBuffer(DID_FLASH_CONFIG, flashCfgString, (uint8_t*)&imxFlashCfg))
        {
            cout << "Failed to parse flash config string." << endl;
            return false;
        }
        if (!inertialSenseInterface.SetImxFlashConfig(imxFlashCfg) ||
            !inertialSenseInterface.WaitForImxFlashCfgSynced())
        {
            cout << "Failed to set IMX flash config." << endl;
            return false;
        }
    }
    else
    {   // Read from flash config
        string output;
        if (cISDataMappings::DidBufferToString(DID_FLASH_CONFIG, (uint8_t*)&imxFlashCfg, output, flashCfgString))
        {
            cout << output;
        }
        else
        {
            cout << "Failed to read flash config." << endl;
        }
    }

    return true;
}

bool cltool_updateGpxFlashCfg(InertialSense& inertialSenseInterface, string flashCfgString)
{
    if (!inertialSenseInterface.WaitForGpxFlashCfgSynced())
    {
        cout << "Failed to sync GPX flash config." << endl;
        return false;
    }
    gpx_flash_cfg_t gpxFlashCfg;
    inertialSenseInterface.GpxFlashConfig(gpxFlashCfg);

    if (flashCfgString.find('=') != std::string::npos)
    {   // Write to flash config
        if (!cISDataMappings::StringToDidBuffer(DID_GPX_FLASH_CFG, flashCfgString, (uint8_t*)&gpxFlashCfg))
        {
            cout << "Failed to parse flash config string." << endl;
            return false;
        }
        g_gpxFlashCfg_upload = gpxFlashCfg; // Save for later use
        if (!inertialSenseInterface.SetGpxFlashConfig(gpxFlashCfg) ||
            !inertialSenseInterface.WaitForGpxFlashCfgSynced())
        {
            cout << "Failed to set GPX flash config." << endl;
            return false;
        }
    }
    else
    {   // Read from flash config
        string output;
        if (cISDataMappings::DidBufferToString(DID_GPX_FLASH_CFG, (uint8_t*)&gpxFlashCfg, output, flashCfgString))
        {
            cout << output;
        }
        else
        {
            cout << "Failed to read flash config." << endl;
        }
    }

    return true;
}
