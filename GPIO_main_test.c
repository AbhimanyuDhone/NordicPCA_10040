//static nrf_drv_timer_t timer = NRF_DRV_TIMER_INSTANCE(0);

//void timer_dummy_handler(nrf_timer_event_t event_type, void * p_context){}
//
//static void led_blinking_setup()
//{
//    uint32_t compare_evt_addr;
//    uint32_t gpiote_task_addr;
//    nrf_ppi_channel_t ppi_channel;
//    ret_code_t err_code;
//    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
//
//    err_code = nrf_drv_gpiote_out_init(GPIO_OUTPUT_PIN_NUMBER, &config);
//    APP_ERROR_CHECK(err_code);
//
//
//    nrf_drv_timer_extended_compare(&timer, (nrf_timer_cc_channel_t)0, 200 * 1000UL, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
//
//    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
//    APP_ERROR_CHECK(err_code);
//
//    compare_evt_addr = nrf_drv_timer_event_address_get(&timer, NRF_TIMER_EVENT_COMPARE0);
//    gpiote_task_addr = nrf_drv_gpiote_out_task_addr_get(GPIO_OUTPUT_PIN_NUMBER);
//
//    err_code = nrf_drv_ppi_channel_assign(ppi_channel, compare_evt_addr, gpiote_task_addr);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = nrf_drv_ppi_channel_enable(ppi_channel);
//    APP_ERROR_CHECK(err_code);
//
//    nrf_drv_gpiote_out_task_enable(GPIO_OUTPUT_PIN_NUMBER);
//}

/**
 * @brief Function for application main entry.
 */
//int main(void)
//{
//    ret_code_t err_code;
//
//    err_code = nrf_drv_ppi_init();
//    APP_ERROR_CHECK(err_code);
//
//    err_code = nrf_drv_gpiote_init();
//    APP_ERROR_CHECK(err_code);
//
//    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
//    err_code = nrf_drv_timer_init(&timer, &timer_cfg, timer_dummy_handler);
//    APP_ERROR_CHECK(err_code);
//#ifdef NRF51
//    //Workaround for PAN-73.
//    *(uint32_t *)0x40008C0C = 1;
//#endif
//
//    // Setup PPI channel with event from TIMER compare and task GPIOTE pin toggle.
//    led_blinking_setup();
//
//    // Enable timer
//    nrf_drv_timer_enable(&timer);
//
//    while (true)
//    {
//        // Do Nothing - GPIO can be toggled without software intervention.
//    }
//}