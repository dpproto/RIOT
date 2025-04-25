/*
 * Copyright (C) 2025 David Picard
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/*
 * @ingroup     sys_auto_init_saul
 * @{
 *
 * @file
 * @brief       Auto initialization for MAX31865 RTD-to-digital converter
 *
 * @author      David Picard
 * @}
 */

#include "assert.h"
#include "log.h"
#include "saul_reg.h"
#include "container.h"
#include "max31865_params.h"
#include "max31865.h"

/**
 * @brief   Define the number of configured devices
 */
#define MAX31865_NUM     ARRAY_SIZE(max31865_params)

/**
 * @brief   Allocate memory for the device descriptors
 */
static max31865_t max31865_devs[MAX31865_NUM];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[MAX31865_NUM * 2];

/**
 * @brief   Define the number of saul info
 */
#define MAX31865_INFO_NUM ARRAY_SIZE(max31865_saul_info)

/**
 * @name    Import SAUL endpoints
 * @{
 */
extern const saul_driver_t max31865_temp_saul_driver;
/** @} */

void auto_init_max31865(void)
{
    assert(MAX31865_INFO_NUM == MAX31865_NUM);

    for (unsigned int i = 0; i < MAX31865_NUM; i++) {
        LOG_DEBUG("[auto_init_saul] initializing MAX31865 #%u\n", i);

        if (max31865_init(&max31865_devs[i], &max31865_params[i]) != 0) {
            LOG_ERROR("[auto_init_saul] error initializing MAX31865 #%u\n", i);
            continue;
        }

        saul_entries[(i * 2)].dev = &(max31865_devs[i]);
        saul_entries[(i * 2)].name = max31865_saul_info[(i * 2)].name;
        saul_entries[(i * 2)].driver = &max31865_temp_saul_driver;

        saul_reg_add(&(saul_entries[(i * 2)]));
        saul_reg_add(&(saul_entries[(i * 2) + 1]));
    }
}
