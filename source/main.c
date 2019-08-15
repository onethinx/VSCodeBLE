
/*******************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Description:
*   This is source code for the PSoC 6 MCU with BLE Find Me code example.
*
* Note:
*
* Owners:
*   snvn@cypress.com
*
* Related Documents:
*   AN210781 - Getting Started with PSoC 6 MCU with
*              Bluetooth Low Energy BLE) Connectivity
*   CE212736 - PSoC 6 MCU with Bluetooth Low Energy
*              (BLE) Connectivity - Find Me Using ModusToolbox
*
* Hardware Dependency:
*  1. PSoC 6 MCU with BLE device
*  2. CY8CKIT-062-BLE Pioneer Kit
*
* Code Tested With:
*  1. ModusToolbox 1.0
*
********************************************************************************
* Copyright 2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

//this should go in modus.mk in order to initialize BLE firmware as single core...
// Software (middleware) components needed by CM0+
//CY_MAINAPP_CM0P_SWCOMP_USED = 
// Software (middleware) components needed by CM4
//CY_MAINAPP_SWCOMP_USED = \
//    $(CY_PSOC_LIB_COMP_MIDDLEWARE_BASE)/ble/config/base \
//    $(CY_PSOC_LIB_COMP_MIDDLEWARE_BASE)/ble/config/single_cm4_softfp

#include "cy_device_headers.h"
#include "cycfg.h"
#include "cycfg_ble.h"
#include "cy_syspm.h"
#include "cy_pdl.h"
#include "DemoKit01.h"
#include "stdio.h"

/***************************************
*       Function Prototypes
***************************************/
void BleFindMe_Init(void);
void BleFindMe_Process(void);
void StackEventHandler(uint32 event, void* eventParam);
void IasEventHandler(uint32 event, void* eventParam);
void EnterLowPowerMode(void);
void ShowError(void);
void PinButtonInterruptHandler( void );

/*******************************************************************************
* Variables to keep track of BLE connection handle
*******************************************************************************/
cy_stc_ble_conn_handle_t    appConnHandle;

/* IAS alert level value. This value is picked up in the main execution loop
   for driving the alert (Blue) LED. */
uint8 alertLevel = 0;
uint8 handler_cnt = 0;
bool handler_flag = false;

/* MCWDT_isr */
const cy_stc_sysint_t MCWDT_isr_cfg = {
    .intrSrc = (IRQn_Type) srss_interrupt_mcwdt_0_IRQn,
    .intrPriority = 7u
};

/* PINBUTTON_isr */
cy_stc_sysint_t PIN_BUTTON_SYSINT_CFG = {
		.intrSrc	   = ioss_interrupts_gpio_0_IRQn,
		.intrPriority  = 0,
};

/* BLESS ISR config for CM4 only */
cy_stc_sysint_t blessIsrCfg = 
{ 
        /* The BLESS interrupt */ 
        .intrSrc = bless_interrupt_IRQn, 
        
        /* The interrupt priority number */ 
        .intrPriority = 1u 
}; 

/*******************************************************************************
* Function Name: BlessInterrupt
****************************************************************************/
static void BlessInterrupt(void)
{
    /* Call interrupt processing */
    Cy_BLE_BlessIsrHandler();
}

/* UART stuff */
cy_stc_scb_uart_context_t uartContext;
char buffer[128];


int main(void)
{
    //Cy_IPC_Sema_Init();
    //Cy_IPC_Pipe_Config();
    //Cy_IPC_Pipe_Init();
    
    init_cycfg_all();

    __enable_irq();

    /* Signal system start */
    for(int i=0;i<20;i++){
        LED_B_INV;
        LED_R_INV;
        CyDelay(200);
    }

    /* Serial */
    Cy_SCB_UART_Init( UART_HW, &UART_config, &uartContext);
	Cy_SCB_UART_Enable( UART_HW );
	Cy_SCB_UART_PutString(UART_HW, "\r-------------------------------------" );
	Cy_SCB_UART_PutString(UART_HW, "\r--  Bluetooth for Onethinx   now!  --" );
	Cy_SCB_UART_PutString(UART_HW, "\r-------------------------------------" );

    /* Initialize BLE */
    Cy_SCB_UART_PutString(UART_HW, "\rInitializing Bluetooth..." );
    BleFindMe_Init();

    /* set up the interrupt handler */
    Cy_SysInt_Init(&PIN_BUTTON_SYSINT_CFG, &PinButtonInterruptHandler );
    NVIC_ClearPendingIRQ(PIN_BUTTON_SYSINT_CFG.intrSrc);
    NVIC_EnableIRQ(PIN_BUTTON_SYSINT_CFG.intrSrc);

    /* has to be called after all the interrupt handlers have been set? */
    __enable_irq();
    
    Cy_SCB_UART_PutString(UART_HW, "\rInitializing Bluetooth done, starting process" );
    for(;;)
    {
        BleFindMe_Process();
        if( handler_flag ){
            sprintf( buffer, "\rhandler_cnt: %d", (int) handler_cnt);
            Cy_SCB_UART_PutString(UART_HW, buffer );
            handler_flag = false;
            
            //test blinking LED by MCDWT:
            NVIC_EnableIRQ(MCWDT_isr_cfg.intrSrc);
            CyDelay(10000);
            NVIC_DisableIRQ(MCWDT_isr_cfg.intrSrc);
        }
    }
}


/*******************************************************************************
* Function Name: MCWDT_Interrupt_Handler
*******************************************************************************/
void MCWDT_Interrupt_Handler(void)
{
    /* Clear the MCWDT peripheral interrupt */
    Cy_MCWDT_ClearInterrupt(MCWDT_HW, CY_MCWDT_CTR0);
    /* Clear the CM4 NVIC pending interrupt for MCWDT */
    NVIC_ClearPendingIRQ(MCWDT_isr_cfg.intrSrc);

    /* If mild alert is received, toggle the Alert LED */
    //if (alertLevel == CY_BLE_MILD_ALERT)
    //{
        Cy_GPIO_Inv(LED_BLUE_PORT, LED_BLUE_PIN);
    //}
}

/*******************************************************************************
* Function Name: BleFindMe_Init()
*******************************************************************************/
void BleFindMe_Init(void)
{
    cy_en_ble_api_result_t          apiResult;
    cy_stc_ble_stack_lib_version_t  stackVersion;

    /* Configure switch SW2 as hibernate wake up source */
    //Cy_SysPm_SetHibWakeupSource(CY_SYSPM_HIBPIN1_LOW); 
    //I don't know where this points to... could be WAKEUP_PIN 0_4 but it's not clear at all

    //the documentation for BLE mentions to set up BLESS interrupt handler - no idea where that is in the example...
    /* Hook interrupt service routines for BLESS */
    (void) Cy_SysInt_Init(&blessIsrCfg, &BlessInterrupt);
    /* Store pointer to blessIsrCfg in BLE configuration structure */
    cy_ble_config.hw->blessIsrConfig = &blessIsrCfg;


    /* Start the UART debug port */
    //UART_DEBUG_START();
    //DEBUG_PRINTF("\r\n\nPSoC 6 MCU with BLE Find Me Code Example \r\n");
    Cy_SCB_UART_PutString(UART_HW, "\rPSoC 6 MCU with BLE Find Me Code Example");

    /* Register the generic event handler */
    Cy_SCB_UART_PutString(UART_HW, "\rRegistering the generic event handler..." );
    Cy_BLE_RegisterEventCallback(StackEventHandler);

    /* Initialize the BLE host */
    Cy_SCB_UART_PutString(UART_HW, "\rInitializing the BLE host..." );
    apiResult = Cy_BLE_Init(&cy_ble_config);

    if(apiResult != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration,
           notify error and halt CPU in debug mode */
        sprintf( buffer, "\rCy_BLE_Init API Error: %x", apiResult );
        Cy_SCB_UART_PutString(UART_HW, buffer );
    }
    else
    {
        sprintf( buffer, "\rCy_BLE_Init API Success: %x", apiResult );
        Cy_SCB_UART_PutString(UART_HW, buffer ); 
    }

    /* Enable BLE */
    Cy_SCB_UART_PutString(UART_HW, "\rEnabling BLE..." );
    apiResult = Cy_BLE_Enable();
    if(apiResult != CY_BLE_SUCCESS)
    {
        /* BLE stack initialization failed, check configuration,
           notify error and halt CPU in debug mode */
        //DEBUG_PRINTF("Cy_BLE_Enable API Error: %x \r\n", apiResult);
        sprintf( buffer, "\rCy_BLE_Enable API Error: %x", apiResult );
        Cy_SCB_UART_PutString(UART_HW, buffer );
        ShowError();
    }
    else
    {
        //DEBUG_PRINTF("Cy_BLE_Enable API Success: %x \r\n", apiResult);
        sprintf( buffer, "\rCy_BLE_Enable API Success: %x", apiResult );
        Cy_SCB_UART_PutString(UART_HW, buffer );
    }

    /* Enable BLE Low Power Mode (LPM) */
    Cy_SCB_UART_PutString(UART_HW, "\rEnabling BLE Low Power Mode (LPM)..." );
    Cy_BLE_EnableLowPowerMode();
    // should this be: ?? Cy_BLE_EnableLPM

    Cy_SCB_UART_PutString(UART_HW, "\rGet Stack version..." );
	apiResult = Cy_BLE_GetStackLibraryVersion(&stackVersion);

    if(apiResult != CY_BLE_SUCCESS)
    {
        //DEBUG_PRINTF("Cy_BLE_GetStackLibraryVersion API Error: 0x%2.2x \r\n", apiResult);
        sprintf( buffer, "\rCy_BLE_GetStackLibraryVersion API Error: 0x%2.2x", apiResult );
        Cy_SCB_UART_PutString(UART_HW, buffer );
        ShowError();
    }    
    else
    {
        //DEBUG_PRINTF("Stack Version: %d.%d.%d.%d \r\n", stackVersion.majorVersion,
        //    stackVersion.minorVersion, stackVersion.patch, stackVersion.buildNumber);
        sprintf( buffer, "\rStack Version: %d.%d.%d.%d", stackVersion.majorVersion, stackVersion.minorVersion, stackVersion.patch, stackVersion.buildNumber);
        Cy_SCB_UART_PutString(UART_HW, buffer );
    }

    /* Register IAS event handler */
    Cy_SCB_UART_PutString(UART_HW, "\rRegistering IAS event handler..." );
    Cy_BLE_IAS_RegisterAttrCallback(IasEventHandler);

    Cy_SCB_UART_PutString(UART_HW, "\rEnabling 4 Hz free-running MCWDT counter 0..." );
    /* Enable 4 Hz free-running MCWDT counter 0*/
    /* MCWDT_config structure is defined by the MCWDT_PDL component based on
       parameters entered in the customizer */
    Cy_MCWDT_Init(MCWDT_HW, &MCWDT_config);
    Cy_MCWDT_Enable(MCWDT_HW, CY_MCWDT_CTR0, 93 /* 2 LFCLK cycles */);
    /* Unmask the MCWDT counter 0 peripheral interrupt */
    Cy_MCWDT_SetInterruptMask(MCWDT_HW, CY_MCWDT_CTR0);

    Cy_SCB_UART_PutString(UART_HW, "\rConfiguring ISR connected to MCWDT interrupt signal..." );
    /* Configure ISR connected to MCWDT interrupt signal*/
    /* MCWDT_isr_cfg structure is defined by the SYSINT_PDL component based on
       parameters entered in the customizer. */
    Cy_SysInt_Init(&MCWDT_isr_cfg, &MCWDT_Interrupt_Handler);
    /* Clear CM4 NVIC pending interrupt for MCWDT */
    NVIC_ClearPendingIRQ(MCWDT_isr_cfg.intrSrc);
    /* Enable CM4 NVIC MCWDT interrupt */
    NVIC_EnableIRQ(MCWDT_isr_cfg.intrSrc);
}

/*******************************************************************************
* Function Name: BleFindMe_Process()
********************************************************************************
*
* Summary:
*   This function processes the BLE events and configures the device to enter
*   low power mode as required.
*
* Parameters:
*  None
*
* Return:
*   None
*
*******************************************************************************/
void BleFindMe_Process(void)
{
    /* The call to EnterLowPowerMode also causes the device to enter hibernate
       mode if the BLE stack is shutdown */
    //EnterLowPowerMode();
    //CyDelay(1);

    /* Cy_Ble_ProcessEvents() allows BLE stack to process pending events */
    Cy_BLE_ProcessEvents();

    /* Update Alert Level value on the Blue LED */
    switch(alertLevel)
    {
        case CY_BLE_NO_ALERT:
            /* Disable MCWDT interrupt at NVIC */
            NVIC_DisableIRQ(MCWDT_isr_cfg.intrSrc);
            /* Turn the Blue LED OFF in case of no alert */
            //Cy_GPIO_Write(Alert_LED_PORT, Alert_LED_PIN, LED_OFF);
            LED_B_SET(LED_OFF);
            break;

        /* Use the MCWDT to blink the Blue LED in case of mild alert */
        case CY_BLE_MILD_ALERT:
            /* Enable MCWDT interrupt at NVIC */
            NVIC_EnableIRQ(MCWDT_isr_cfg.intrSrc);
            /* The MCWDT interrupt handler will take care of LED blinking */
            break;

        case CY_BLE_HIGH_ALERT:
            /* Disable MCWDT interrupt at NVIC */
            NVIC_DisableIRQ(MCWDT_isr_cfg.intrSrc);
            /* Turn the Blue LED ON in case of high alert */
            //Cy_GPIO_Write(Alert_LED_PORT, Alert_LED_PIN, LED_ON);
            LED_B_SET(LED_ON);
            break;

        /* Do nothing in all other cases */
        default:
            break;
    }
}

/*******************************************************************************
* Function Name: StackEventHandler()
********************************************************************************
*
* Summary:
*   This is an event callback function to receive events from the BLE Component.
*
* Parameters:
*  uint32 event:      event from the BLE component
*  void* eventParam:  parameters related to the event
*
* Return:
*   None
*
*******************************************************************************/
void StackEventHandler(uint32 event, void* eventParam)
{
    cy_en_ble_api_result_t      apiResult;
    uint8 i;

    switch (event)
	{
        /* There are some events generated by the BLE component
        *  that are not required for this code example. */

        /**********************************************************
        *                       General Events
        ***********************************************************/
		/* This event is received when the BLE stack is started */
        case CY_BLE_EVT_STACK_ON:
            //DEBUG_PRINTF("CY_BLE_EVT_STACK_ON, Start Advertisement \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_STACK_ON, Start Advertisement" );
            /* Enter into discoverable mode so that remote device can search it */
            apiResult = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            if(apiResult != CY_BLE_SUCCESS)
            {
                //DEBUG_PRINTF("Start Advertisement API Error: %d \r\n", apiResult);
                sprintf( buffer, "\rStart Advertisement API Error: %d", apiResult);
                Cy_SCB_UART_PutString(UART_HW, buffer );
                ShowError();
                /* Execution does not continue beyond this point */
            }
            else
            {
                //DEBUG_PRINTF("Start Advertisement API Success: %d \r\n", apiResult);
                //Cy_GPIO_Write(Advertising_LED_PORT, Advertising_LED_PIN, LED_ON);
                //Cy_GPIO_Write(Disconnect_LED_PORT, Disconnect_LED_PIN, LED_OFF);
                LED_R_SET(LED_ON);
                sprintf( buffer, "\rStart Advertisement API Success: %d", apiResult);
                Cy_SCB_UART_PutString(UART_HW, buffer );
                alertLevel = CY_BLE_NO_ALERT;
            }

            /* Get address of the device */
            apiResult = Cy_BLE_GAP_GetBdAddress();
            if(apiResult != CY_BLE_SUCCESS)
            {
                //DEBUG_PRINTF("Cy_BLE_GAP_GetBdAddress API Error: %d \r\n", apiResult);
                sprintf( buffer, "\rCy_BLE_GAP_GetBdAddress API Error: %d", apiResult);
                Cy_SCB_UART_PutString(UART_HW, buffer );
            }
            else
            {
                //DEBUG_PRINTF("Cy_BLE_GAP_GetBdAddress API Success: %d \r\n", apiResult);
                sprintf( buffer, "\rCy_BLE_GAP_GetBdAddress API Success: %d", apiResult);
                Cy_SCB_UART_PutString(UART_HW, buffer );
            }

            break;

        /* This event is received when there is a timeout */
        case CY_BLE_EVT_TIMEOUT:
            //DEBUG_PRINTF("CY_BLE_EVT_TIMEOUT \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_TIMEOUT");
            break;

        /* This event indicates that some internal HW error has occurred */
		case CY_BLE_EVT_HARDWARE_ERROR:
            //DEBUG_PRINTF("Hardware Error \r\n");
            ShowError();
            Cy_SCB_UART_PutString(UART_HW, "\rHardware Error");
			break;

        /*  This event will be triggered by host stack if BLE stack is busy or
         *  not busy. Parameter corresponding to this event will be the state
    	 *  of BLE stack.
         *  BLE stack busy = CYBLE_STACK_STATE_BUSY,
    	 *  BLE stack not busy = CYBLE_STACK_STATE_FREE
         */
    	case CY_BLE_EVT_STACK_BUSY_STATUS:
            //DEBUG_PRINTF("CY_BLE_EVT_STACK_BUSY_STATUS: %x\r\n", *(uint8 *)eventParam);
            sprintf( buffer, "\rCY_BLE_EVT_STACK_BUSY_STATUS: %x", *(uint8 *)eventParam);
            Cy_SCB_UART_PutString(UART_HW, buffer );
            break;

        /* This event indicates completion of Set LE event mask */
        case CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE:
            //DEBUG_PRINTF("CY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_LE_SET_EVENT_MASK_COMPLETE");
            break;

        /* This event indicates set device address command completed */
        case CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE:
            //DEBUG_PRINTF("CY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_SET_DEVICE_ADDR_COMPLETE ");
            break;

        /* This event indicates get device address command completed
           successfully */
        case CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE:
            //DEBUG_PRINTF("CY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE: ");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_GET_DEVICE_ADDR_COMPLETE: ");
            for(i = CY_BLE_GAP_BD_ADDR_SIZE; i > 0u; i--)
            {
                //DEBUG_PRINTF("%2.2x", ((cy_stc_ble_bd_addrs_t *)((cy_stc_ble_events_param_generic_t *)eventParam)->eventParams)->publicBdAddr[i-1]);
                sprintf( buffer, "\r%2.2x", ((cy_stc_ble_bd_addrs_t *)((cy_stc_ble_events_param_generic_t *)eventParam)->eventParams)->publicBdAddr[i-1]);
                Cy_SCB_UART_PutString(UART_HW, buffer );
            }
            //DEBUG_PRINTF("\r\n");
            break;

        /* This event indicates set Tx Power command completed */
        case CY_BLE_EVT_SET_TX_PWR_COMPLETE:
            //DEBUG_PRINTF("CY_BLE_EVT_SET_TX_PWR_COMPLETE \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_SET_TX_PWR_COMPLETE");
            break;

        /* This event indicates that stack shutdown is complete */
        case CY_BLE_EVT_STACK_SHUTDOWN_COMPLETE:
            //DEBUG_PRINTF("CY_BLE_EVT_STACK_SHUTDOWN_COMPLETE \r\n");
            //DEBUG_PRINTF("Entering hibernate mode \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_STACK_SHUTDOWN_COMPLETE");
            Cy_SCB_UART_PutString(UART_HW, "\rEntering hibernate mode");
            CyDelay(10);
            //DEBUG_WAIT_UART_TX_COMPLETE();
            Cy_SysPm_Hibernate();
            ShowError();
            /* Code execution will not reach here */
            /* Device wakes up from hibernate and performs reset sequence
               when the reset switch or SW2 switch on the kit is pressed */
            break;

        /**********************************************************
        *                       GAP Events
        ***********************************************************/

        /* This event indicates peripheral device has started/stopped
           advertising */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            //DEBUG_PRINTF("CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP: ");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP: ");

            if(Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_ADVERTISING)
            {
                //DEBUG_PRINTF("Advertisement started \r\n");
                //Cy_GPIO_Write(Advertising_LED_PORT, Advertising_LED_PIN, LED_ON);
                //Cy_GPIO_Write(Disconnect_LED_PORT, Disconnect_LED_PIN, LED_OFF);
                Cy_SCB_UART_PutString(UART_HW, "\rAdvertisement started");
                LED_R_SET(LED_ON);
            }
            else if(Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_STOPPED)
            {
                //DEBUG_PRINTF("Advertisement stopped \r\n");
                //Cy_GPIO_Write(Advertising_LED_PORT, Advertising_LED_PIN, LED_OFF);
                //Cy_GPIO_Write(Disconnect_LED_PORT, Disconnect_LED_PIN, LED_ON);
                Cy_SCB_UART_PutString(UART_HW, "\rAdvertisement stopped");
                LED_R_SET(LED_OFF);
                /* Advertisement event timed out before connection, shutdown BLE
                * stack to enter hibernate mode and wait for device reset event
                * or SW2 press to wake up the device */
                Cy_BLE_Disable();
            }
            break;

        /* This event is generated at the GAP Peripheral end after connection
           is completed with peer Central device */
        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
            //DEBUG_PRINTF("CY_BLE_EVT_GAP_DEVICE_CONNECTED \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_GAP_DEVICE_CONNECTED");
            break;

        /* This event is generated when disconnected from remote device or
           failed to establish connection */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            if(Cy_BLE_GetConnectionState(appConnHandle) == CY_BLE_CONN_STATE_DISCONNECTED)
            {
                //DEBUG_PRINTF("CY_BLE_EVT_GAP_DEVICE_DISCONNECTED %d\r\n", CY_BLE_CONN_STATE_DISCONNECTED);
                sprintf( buffer, "\rCY_BLE_EVT_GAP_DEVICE_DISCONNECTED %d", CY_BLE_CONN_STATE_DISCONNECTED);
                Cy_SCB_UART_PutString(UART_HW, buffer );
            
                alertLevel = CY_BLE_NO_ALERT;

                //Cy_GPIO_Write(Advertising_LED_PORT, Advertising_LED_PIN, LED_OFF);
                //Cy_GPIO_Write(Disconnect_LED_PORT, Disconnect_LED_PIN, LED_ON);
                LED_R_SET(LED_OFF);
                /* Enter into discoverable mode so that remote device can search it */
                apiResult = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
                if(apiResult != CY_BLE_SUCCESS)
                {
                    //DEBUG_PRINTF("Start Advertisement API Error: %d \r\n", apiResult);
                    sprintf( buffer, "\rStart Advertisement API Error: %d", apiResult);
                    Cy_SCB_UART_PutString(UART_HW, buffer );
                    ShowError();
                    /* Execution does not continue beyond this point */
                }
                else
                {
                    //DEBUG_PRINTF("Start Advertisement API Success: %d \r\n", apiResult);
                    //Cy_GPIO_Write(Advertising_LED_PORT, Advertising_LED_PIN, LED_ON);
                    //Cy_GPIO_Write(Disconnect_LED_PORT, Disconnect_LED_PIN, LED_OFF);
                    sprintf( buffer, "\rStart Advertisement API Success: %d", apiResult);
                    Cy_SCB_UART_PutString(UART_HW, buffer );
                    LED_R_SET(LED_ON);
                }

            }
            break;

        /* This event is generated at the GAP Central and the peripheral end
           after connection parameter update is requested from the host to
           the controller */
        case CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE:
            //DEBUG_PRINTF("CY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE");
            break;

        /* This event is triggered instead of 'CY_BLE_EVT_GAP_DEVICE_CONNECTED',
           if Link Layer Privacy is enabled in component customizer */
        case CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE:

            /* BLE link is established */
            /* This event will be triggered since link layer privacy is enabled */
            //DEBUG_PRINTF("CY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_GAP_ENHANCE_CONN_COMPLETE");
            if(Cy_BLE_GetState() == CY_BLE_STATE_ON)
            {
                //Cy_GPIO_Write(Advertising_LED_PORT, Advertising_LED_PIN, LED_OFF);
                //Cy_GPIO_Write(Disconnect_LED_PORT, Disconnect_LED_PIN, LED_OFF);
                LED_R_SET(LED_OFF);
            }
            break;

        /**********************************************************
        *                       GATT Events
        ***********************************************************/

        /* This event is generated at the GAP Peripheral end after connection
           is completed with peer Central device */
        case CY_BLE_EVT_GATT_CONNECT_IND:
            appConnHandle = *(cy_stc_ble_conn_handle_t *)eventParam;
            //DEBUG_PRINTF("CY_BLE_EVT_GATT_CONNECT_IND: %x, %x \r\n",
            //            (*(cy_stc_ble_conn_handle_t *)eventParam).attId,
            //            (*(cy_stc_ble_conn_handle_t *)eventParam).bdHandle);
            sprintf( buffer, "\rCY_BLE_EVT_GATT_CONNECT_IND: %x, %x", (*(cy_stc_ble_conn_handle_t *)eventParam).attId, (*(cy_stc_ble_conn_handle_t *)eventParam).bdHandle);
            Cy_SCB_UART_PutString(UART_HW, buffer );
            break;

        /* This event is generated at the GAP Peripheral end after
           disconnection */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
            //DEBUG_PRINTF("CY_BLE_EVT_GATT_DISCONNECT_IND \r\n");
            //Cy_GPIO_Write(Disconnect_LED_PORT, Disconnect_LED_PIN, LED_ON);
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_GATT_DISCONNECT_IND");
            break;

        /* This event is triggered when 'GATT MTU Exchange Request'
           received from GATT client device */
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
            //DEBUG_PRINTF("CY_BLE_EVT_GATTS_XCNHG_MTU_REQ \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_GATTS_XCNHG_MTU_REQ");
            break;

        /* This event is triggered when a read received from GATT
           client device */
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
            //DEBUG_PRINTF("CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ \r\n");
            Cy_SCB_UART_PutString(UART_HW, "\rCY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ");
            break;

        /**********************************************************
        *                       Other Events
        ***********************************************************/
        default:
            //DEBUG_PRINTF("Other event: %lx \r\n", (unsigned long) event);
            sprintf( buffer, "\rOther event: %lx", (unsigned long) event);
            Cy_SCB_UART_PutString(UART_HW, buffer );
			break;
	}
}

/*******************************************************************************
* Function Name: IasEventHandler
********************************************************************************
*
* Summary:
*  This is an event callback function to receive events from the BLE Component,
*  which are specific to Immediate Alert Service.
*
* Parameters:
*  uint32 event:      event from the BLE component
*  void* eventParams: parameters related to the event
*
* Return:
*  None
*
*******************************************************************************/
void IasEventHandler(uint32 event, void *eventParam)
{
    /* Alert Level Characteristic write event */
    if(event == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Read the updated Alert Level value from the GATT database */
        Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL,
            sizeof(alertLevel), &alertLevel);
    }

}

/*******************************************************************************
* Function Name: EnterLowPowerMode()
********************************************************************************
* Summary:
*  Configures the device to enter low power mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Theory:
*  The function configures the device to enter deep sleep - whenever the
*  BLE is idle and the UART transmission/reception is not happening.
*
*  In case of disconnection, the function configures the device to
*  enter hibernate mode.
*
*******************************************************************************/
void EnterLowPowerMode(void)
{
    cy_en_syspm_status_t retval = CY_SYSPM_SUCCESS;
    //DEBUG_PRINTF("Entering deep sleep mode \r\n");
    //DEBUG_WAIT_UART_TX_COMPLETE();
    Cy_SCB_UART_PutString(UART_HW, "\rEntering deep sleep mode");
    CyDelay(1);
    /* Configure deep sleep mode to wake up on interrupt */
    //Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

    //The above may just try to put CM0 in sleep as well. The code below just puts CM4 to sleep...
    // It appears the the BLE registers some callback that gets executed before going to sleep
    // One of these callbacks is locking the system: Cy_BLE_DeepSleepCallbackSingleCore
    // 
    retval = Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    if( retval != CY_SYSPM_SUCCESS ) {
            sprintf( buffer, "\rfailed to go to sleep: %x", (int) retval );
            Cy_SCB_UART_PutString(UART_HW, buffer );
	}
    
    //CyDelay(2000); //Above function fails after second attempt

    //DEBUG_PRINTF("Exiting deep sleep mode \r\n");
    //DEBUG_WAIT_UART_TX_COMPLETE();
    Cy_SCB_UART_PutString(UART_HW, "\rExiting deep sleep mode");
    sprintf( buffer, "\rhandler_cnt: %d", (int) handler_cnt);
    Cy_SCB_UART_PutString(UART_HW, buffer );
    CyDelay(1);
}

/*******************************************************************************
* Indicate an error to the user; endless loop
*******************************************************************************/

void ShowError(void)
{
    for(;;){
                LED_R_INV;
                LED_B_INV;
                CyDelay(50);
            }
}

/*******************************************************************************
* Summary:
*   handles an interrupt on PIN_BUTTON (pin 0.4).
*   see design.modus for details on pin configuration.
*   note that a single button press may invoke this function more than once.
*******************************************************************************/
void PinButtonInterruptHandler( void ){
	/* First thing to do is to clear the interrupt, otherwise it will retrigger */
	Cy_GPIO_ClearInterrupt( PIN_BUTTON_PORT, PIN_BUTTON_PIN );
	handler_cnt++;
    handler_flag = true;
}