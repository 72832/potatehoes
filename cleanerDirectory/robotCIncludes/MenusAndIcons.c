///////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                            Menus and Icons
//
// This file contains declarations that provide easy support for drawing menus and ICONS on the
// NXT LCD.
//
// The file should be added to an application program via an "#include" directive.
//
///////////////////////////////////////////////////////////////////////////////////////////////////
#pragma systemFile      // this eliminates warning for "unreferenced" functions

typedef enum TMenuAnimationType
{
  menuRefreshComplete,   // Internal state, not to be used by end user application calls.
  menuOverwrite,
  menuFromLeft,
  menuFromRight,
  menuFromTop,
  menuFromBottom,
} TMenuAnimationType;

typedef enum TFiletypes                   // File type id's
{
  FILETYPE_ALL,       //  0 = All
  FILETYPE_SOUND,     //  1 = Sound
  FILETYPE_LMS,       //  2 = LMS
  FILETYPE_NXT,       //  3 = NXT               -- "prg" On-Brick-Program file
  FILETYPE_TRYME,     //  4 = Try me
  FILETYPE_DATALOG,   //  5 = Datalog
  FILETYPE_TEMP,      //  6 = Temp files
  FILETYPE_TEXT,      //  7 = Text files
  FILETYPE_ICON,      //  8 = Icon (RIC) files
  FILETYPE_SYS,       //  9 = System file (e.g. rpgReader.sys)
  FILETYPE_MENU,      // 10 = Menu file
  FILETYPES
} TFiletypes;


typedef enum TDeviceTypeIcon
{
  DEVICETYPE_ANY,
  DEVICETYPE_NXT,
  DEVICETYPE_PHONE,
  DEVICETYPE_PC
} TDeviceTypeIcon;

typedef enum TIconIndex
{
  iconNone                = 0x00,
  // 0x01                          // Empty
  iconSoundDB             = 0x02,
  iconSoundDBA            = 0x03,
  iconLightReflected      = 0x04,
  iconLightAmbient        = 0x05,
  iconLightSensor         = 0x06,
  iconTouch               = 0x07,
  iconMotorDegrees        = 0x08,
  iconMotorRotation       = 0x09,
  iconRotation            = 0x0A,
  iconUltrasonicInch      = 0x0B,
  iconUltrasonicCm        = 0x0C,
  iconTemperatureC        = 0x0D,
  iconTemperatureF        = 0x0E,
  // 0X0F                         // Empty

  // 0X10                         // Empty
  // 0X11                         // Empty
  iconPort1               = 0x12,
  iconPort2               = 0x13,
  iconPort3               = 0x14,
  iconPort4               = 0x15,
  iconPortA               = 0x16,
  iconPortB               = 0x17,
  iconPortC               = 0x18,
  // 0X19                         // Empty
  iconFileBase            = 0x1A,
  iconFileTypeAny         = iconFileBase + FILETYPE_ALL,
  iconFileTypeSound       = iconFileBase + FILETYPE_SOUND,         // 0x1B
  iconFileTypeExeFiles    = iconFileBase + FILETYPE_LMS,           // 0x1C
  iconFileTypeOnBrickPrgm = iconFileBase + FILETYPE_NXT,           // 0x1D
  iconFileTypeTryMe       = iconFileBase + FILETYPE_TRYME,         // 0x1E
  iconFileTypeDatalog     = iconFileBase + FILETYPE_DATALOG,       // 0x1F
  iconFileTypeTemp        = iconFileBase + 0,   // No icons have been defined for these
  iconFileTypeText        = iconFileBase + 0,   // No icons have been defined for these
  iconFileTypeIcon        = iconFileBase + 0,   // No icons have been defined for these
  iconFileTypeSys         = iconFileBase + 0,   // No icons have been defined for these

  // 0x20                          // Empty
  iconEmpty               = 0x21,
  iconForward             = 0x22,
  iconForward5            = 0x23,
  iconBackLeft2           = 0x24,  // Back left 2 rotations
  iconTurnLeft            = 0x25,
  iconTurnLeft2           = 0x26,
  iconBackRight           = 0x27,
  iconTurnRight           = 0x28,
  iconTurnRight2          = 0x29,
  iconBackLeft            = 0x2A,
  iconTone1               = 0x2B,
  iconTone2               = 0x2C,
  iconBackward            = 0x2D,
  iconBackward5           = 0x2E,
  iconBackRight2          = 0x2F,

  iconNo                  = 0x30,
  iconYes                 = 0x31,
  iconRun                 = 0x32,
  iconSend                = 0x33,
  iconDelete              = 0x34,
  iconIncrement           = 0x35,
  iconDecrement           = 0x36,
  iconMainMenu            = 0x37, // Revert to "main menu" for On Brick Programming -- 180 degree "arrow"
  icon0x38                = 0x38, // Some kind of stopwatch or alarm clock
  iconVolume              = 0x39,
  iconSleep               = 0x3A,
  iconMyFiles             = 0x3B,
  iconOnBrickProgramming  = 0x3C,
  iconDataGraph           = 0x3D, // May be Unused ????
  iconSensorView          = 0x3E,
  iconSettings            = 0x3F,

  iconTryMe               = 0x40,
  iconEmptyOBP            = 0x41, // Duplicate
  iconLight               = 0x42,
  iconObject              = 0x43,
  iconSound               = 0x44,
  iconTouchOBP            = 0x45, // Duplicate
  iconWait2               = 0x46,
  iconWait5               = 0x47,
  iconWait10              = 0x48,
  iconDark                = 0x49,
  iconINVALID             = 0x4A, // Empty
  // 0x4B                         // Empty
  // 0x4C                         // Empty
  iconStop                = 0x4D, // 61
  iconLoop                = 0x4E, // 62
  iconVersion             = 0x4F, // 63

  // 0x50
  iconBluetooth           = 0x51,
  iconBTContacts          = 0x52,
  iconBTConnections       = 0x53,
  iconBTVisibility        = 0x54,
  iconBTOnOff             = 0x55,
  iconBTSearch            = 0x56, // Search "Magnifying glass"
  iconBTSearching         = 0x57, // Bluetooth ICON with "?"
  iconBTConnectDevice     = 0x58, // Bluetooth ICON with ">"
  iconBTDisconnect        = 0x59,
  iconBTVisible           = 0x5A, // Same as "iconBTVisibility"
  iconBTInvisible         = 0x5B,
  iconBTOn                = 0x5C,  // Same as "iconBTOnOff"
  iconBTOff               = 0x5D,

  //
  // Graphics icons for Connection Ports -- 0, 1, 2, 3
  //
  iconBTConnectionPort0   = 0x70,
  iconBTConnectionPort1   = iconBTConnectionPort0 + 1,
  iconBTConnectionPort2   = iconBTConnectionPort0 + 2,
  iconBTConnectionPort3   = iconBTConnectionPort0 + 3,
  //
  // Graphics icons for Device Types -- NXT, Phone, PC
  //
  iconBTDeviceTypeBase    = 0x74,
  iconBTDeviceTypeUnknown = iconBTDeviceTypeBase + DEVICETYPE_ANY,
  iconBTDeviceTypeNXT     = iconBTDeviceTypeBase + DEVICETYPE_NXT,
  iconBTDeviceTypePhone   = iconBTDeviceTypeBase + DEVICETYPE_PHONE,
  iconBTDeviceTypePC      = iconBTDeviceTypeBase + DEVICETYPE_PC,
} TIconIndex;


intrinsic void drawMenu(TMenuAnimationType nAnimation,
                        const TIconIndex leftIcon,
                        const TIconIndex centerIcon,
                        const TIconIndex rightIcon,
                        const string sIconText,
                        const string sHeadingText = "")
                    asm(opcdSystemFunctions, byte(sysFuncDrawNXTMenu),
                        variable(nAnimation),
                        variable(leftIcon),
                        variable(centerIcon),
                        variable(rightIcon),
                        variableRefString(sIconText),
                        variableRefString(sHeadingText));

intrinsic bool bValidIcon(TIconIndex nIconIndex)
                    asm(opcdSystemFunctions, byte(sysFuncValidIcon), variable(nIconIndex), functionReturn);
