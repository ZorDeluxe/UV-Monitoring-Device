
#include "eeprom_module.h"


// Defines the TWI/I2C instance to use.
const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);


/****************************************************
 * Write data to EEPROM.
 * Input parameters for address, size of data to 
 * write, and pointer to data.
 ***************************************************/
ret_code_t eeprom_write(uint8_t w_0, uint8_t w_1, uint8_t w_2, uint8_t const * pdata, size_t size)
{
    ret_code_t ret;
    uint16_t addr16 = ((w_2 << 8) | w_1);
    /* Memory device supports only a limited number of bytes written in sequence */
    if (size > (EEPROM_SEQ_WRITE_MAX_BYTES))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    /* All written data must be in the same page */
    if ((addr16 / (EEPROM_SEQ_WRITE_MAX_BYTES)) != ((addr16 + size - 1) / (EEPROM_SEQ_WRITE_MAX_BYTES)))
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    do
    {
        uint8_t buffer[EEPROM_ADDRESS_LEN_BYTES + EEPROM_SEQ_WRITE_MAX_BYTES]; /* Addr + data */
        uint8_t  addr8  = (EEPROM_ADDR | (w_0 << 1));          // Sets the EEPROM MSB Address

        memcpy(buffer, &addr16, EEPROM_ADDRESS_LEN_BYTES);
        memcpy(buffer + EEPROM_ADDRESS_LEN_BYTES, pdata, size);

        ret = nrf_drv_twi_tx(&m_twi_master, addr8, buffer, size + EEPROM_ADDRESS_LEN_BYTES, false);
    }while (0);
    return ret;
}


/****************************************************
 * Read data from EEPROM.
 * Input parameters for address, size of data to 
 * read, and pointer to store data.
 ***************************************************/
ret_code_t eeprom_read(uint8_t w_0, uint8_t w_1, uint8_t w_2, uint8_t * pdata, size_t size)
{
    ret_code_t ret;
//    if (size > (EEPROM_SIZE))
//    {
//        return NRF_ERROR_INVALID_LENGTH;
//    }
    do
    {
       uint16_t addr16 = ((w_2 << 8) | w_1);
       uint8_t  addr8  = (EEPROM_ADDR | (w_0 << 1));          // Sets the EEPROM MSB Address

       ret = nrf_drv_twi_tx(&m_twi_master, addr8, (uint8_t *)&addr16, EEPROM_ADDRESS_LEN_BYTES, true);
       if (NRF_SUCCESS != ret)
       {
           break;
       }
       ret = nrf_drv_twi_rx(&m_twi_master, addr8, pdata, size);
    }while (0);
    return ret;
}


/****************************************************
 * Initialise TWI/I2C module.
 ***************************************************/
ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}


/****************************************************
 * Initialises circular buffer.
 * Just sets the write address to be 0x14.
 ***************************************************/
ret_code_t init_circ_buf(void)
{
    ret_code_t  ret;
    
    uint8_t     r_0 = 0b00;
    uint8_t     r_1 = 0b00000000;
    uint8_t     r_2 = 0b00000000;
    
    uint32_t w_index = 0x14;

    ret = eeprom_write(r_0, r_1, r_2, (uint8_t *) &w_index, 3);

    return ret;
}


/****************************************************
 * Write to circular buffer.
 * First reads address to write to, and then writes.
 ***************************************************/
ret_code_t eeprom_write_circ_buf(uint8_t * value)
{
    ret_code_t  ret = NRF_SUCCESS;
    uint8_t     r_0 = 0b00;
    uint8_t     r_1 = 0b00000000;
    uint8_t     r_2 = 0b00000000;

    uint8_t     w_0;
    uint8_t     w_1;
    uint8_t     w_2;
    uint16_t    i = 0;

    uint32_t    w_index = 20;

    // Read next write address
    while (eeprom_read(r_0, r_1, r_2, (uint8_t *) &w_index, 3) != NRF_SUCCESS)
    {
        eeprom_wait();
    }

    NRF_LOG_INFO("Write Index: %d", w_index);

    w_0 = (((0b00000011 << 16) & w_index) >> 16);
    w_1 = (((0b11111111 << 8) & w_index) >> 8);
    w_2 = ((0b11111111) & w_index);

    while (eeprom_write(w_0, w_1, w_2, value, 1) != NRF_SUCCESS)
    {
        eeprom_wait();
    }
    
    // Updates the write index and stores in EEPROM.
    w_index += 1;
    if (w_index > MAX_ADDRESS)
    {
      w_index = 0x14;
    }
    // Writes the new write index address back to the EEPROM
    while (eeprom_write(r_0, r_1, r_2, (uint8_t *) &w_index, 3) != NRF_SUCCESS)
    {
        eeprom_wait();
    }

    return ret;
}


/****************************************************
 * Read SED from EEPROM.
 ***************************************************/
ret_code_t eeprom_read_dosage(uint8_t * value)
{
    ret_code_t  ret = NRF_SUCCESS;
    
    uint8_t     r_0 = 0b00;
    uint8_t     r_1 = 0b00000000;
    uint8_t     r_2 = 0b00000100;
    
    while (ret = eeprom_read(r_0, r_1, r_2, value, 4) != NRF_SUCCESS)
    {
      eeprom_wait();
    }

    return ret;
}


/****************************************************
 * Update the SED in EEPROM
 ***************************************************/
ret_code_t eeprom_write_dosage(uint8_t * value)
{
    ret_code_t  ret = NRF_SUCCESS;
    
    uint8_t     w_0 = 0b00;
    uint8_t     w_1 = 0b00000000;
    uint8_t     w_2 = 0b00000100;

    while (eeprom_write(w_0, w_1, w_2, value, 4) != NRF_SUCCESS)
    {
        eeprom_wait();
    }

    return ret;
}


/****************************************************
 * Continuously pings the EEPROM until it ACKs.
 * Required due to the fact that there is a non-zero
 * period between writes.
 ***************************************************/
void eeprom_wait(void)
{
    while (nrf_drv_twi_tx(&m_twi_master, EEPROM_ADDR, NULL, 0, false) == 33281)
    {
      //NRF_LOG_INFO("i2c_err");
    }
    return;
}