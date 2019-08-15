# Onethinx Bluetooth example

# Description

Based on the modus 1.0 BLE code example (CE212736).
I took out debug.h/c and led.h and all references to functions therein. 
I added pdl_mw/ble directories to the CMakeLists in an attempt to make it build.
Taken out the BLEFindME.h/c and put everything in main.
Adjusted pins to the Onethinx module.
Stepped through code; error at Cy_BLE_Enable()->Cy_BLE_EnableHost()->Cy_BLE_StackInit(&stackInitParam).
Modified cy_syslib.h at line 978 (THIS IS IN THE PDL):
``` c
__STATIC_INLINE uint8_t Cy_SysLib_GetDeviceRevision(void)
{
    //return ((SFLASH_SI_REVISION_ID == 0UL) ? CY_SYSLIB_DEVICE_REV_0A : SFLASH_SI_REVISION_ID);
    //JAN INSERT!!!
    return CY_SYSLIB_DEVICE_REV_0A;
}
```
It appears that the above code tries to access SFLASH in some way, and that is not allowed.
Added LED and UART output to the remainder of the functions.
The blue LED takes the role of ALERTLED, the red LED takes the role of ADVERTISINGLED.
Execution stops at second call of EnterLowPowerMode->Cy_SysPm_DeepSleep->Cy_SysPm_ExecuteCallback-> Cy_BLE_DeepSleepCallbackSingleCore->Cy_IPC_Sema_Status.
Commenting out the LowPowerMode stuff will make the whole thing discoverable by the CySmart 5677 dongle.

ipcChannel = 4
cyBLESema = 3
Cy_Ipc_SemaStruct = 0x40230080
SemaStruct = 0x80404ec

Added the switch code of SendOnSwitch-example.
Using a dummy stack, without Secure_Init() indeed solves the Sema-issue.
Make sure that the BLE_ECO clock is not input for FLL (prevents low power mode).
Make sure the CLK_LF is activated (BLE_ENABLE will fail otherwise).
LowPowerMode seems to work.
cy_syslib hack can be reverted.
I checked the 4Hz counter, it works (probably never gets activated).

Now the "Fine me target" is discoverable with the dongle. However, i cannot connect to it.

It appears that the BLESS interrupt handler was never set-up in the example code.
I added it as per Cypress' own documentation.
Connecting now works. In CySmart, do 'Discover all attributes', find 'immediate alert' and 'Alert level'.
On the far right side, enter 0, 1, 2 and press 'write value without response'. The blue LED will be off, blinking, on.


LEDS:
1. RED - ADEVERTISING STATUS, set in StackEventHandler()
  * ON    - Advertising started
  * OFF   - Advertising stopped
1. BLUE - ALERT LEVEL, set in BleFindMe_Process()
  * ON    - CY_BLE_HIGH_ALERT
  * BLINK - CY_BLE_MILD_ALERT
  * OFF   - CY_BLE_NO_ALERT
1. BOTH
  * BLINKING FAST - ShowError()

# TODO

1. ?
