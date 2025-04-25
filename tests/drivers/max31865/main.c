/*
 * Copyright (C) 2025 David Picard
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the MAX31865 RTD-to-Digital converter driver
 *
 * @author      David Picard
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "max31865.h"
#include "max31865_params.h"
#include "ztimer.h"

#define MEASUREMENT_SLEEP_MS        (1000)
#define MAX_LOOPS_MEAS          (3)

/* supported acquisition modes */
enum {
    MAX31865_NORMAL,
    MAX31865_SAUL
} acqmode = MAX31865_NORMAL;


void print_fault(max31865_fault_t value);

int main(void)
{
    max31865_t dev;
    max31865_data_t data;
    phydat_t phyTemp;
    int cntMeas = 0;
    saul_reg_t *saulDev = saul_reg;
    phydat_t saulData;

    puts("MAX31865 RTD-to-Digital converter test application\n");

    /* initialize first configured sensor */
    printf("Initializing MAX31865 converter...\t");
    if (max31865_init(&dev, &max31865_params[0]) == 0) {
        puts("[OK]\n");
    }
    else {
        puts("[Failed]");
        return 1;
    }

    puts("------------------------------");
    puts("Switch to normal mode");

    /* periodically convert temperature values */
    while (1) {
        ztimer_sleep(ZTIMER_MSEC, MEASUREMENT_SLEEP_MS);

        switch(acqmode) {
            case MAX31865_NORMAL:
                max31865_read(&dev, &data);
                phyTemp.val[0] = data.rtd_temperature_cdegc;    /* rtd_temperature_cdegc is in 0.01°C */
                phyTemp.scale = -2;             /* 1 cdegC = 1e-02 degC */
                phyTemp.unit = UNIT_TEMP_C;     /* set the unit */
                phydat_dump(&phyTemp, 1);       /* print the value in a pretty format */
                break;
            case MAX31865_SAUL:
                while (saulDev) {
                    int dim = saul_reg_read(saulDev, &saulData);
                    printf("\nDev: %s\tType: %s" "\n", saulDev->name,
                            saul_class_to_str(saulDev->driver->type));
                    phydat_dump(&saulData, dim);
                    saulDev = saulDev->next;
                }
                saulDev = saul_reg; /* reset pointer for next read */
                break;
        }

        if (data.fault != MAX31865_FAULT_NO_FAULT) {
            print_fault(data.fault);
            continue;
        }

        /* Switch modes periodically: */
        cntMeas++;
        if (cntMeas == MAX_LOOPS_MEAS) {
            cntMeas = 0;
            acqmode++;
            if (acqmode > MAX31865_SAUL) {
                acqmode = 0;
            }
            puts("------------------------------");
            switch (acqmode) {
                case MAX31865_NORMAL:
                    puts("Switch to normal mode");
                    break;
                case MAX31865_SAUL:
                    puts("Switch to MAX31865_SAUL mode");
                    break;
                default:
                    acqmode = 0;
                    puts("ERROR: unsupported mode");
            }
            puts("------------------------------");
        }

        // /* format strings are defined in inttypes.h */
        // printf("Temperature: %" PRIi32 ".%"PRId32 "°C\n",
        //        data.rtd_temperature_cdegc/100, abs(data.rtd_temperature_cdegc%100));    }

    return 0;
    }
}


void print_fault(max31865_fault_t value)
{
    switch (value) {
        case MAX31865_FAULT_NO_FAULT:
            break;
        case MAX31865_FAULT_CIRCUIT:
            puts("Fault: Short or open circuit");
            break;
        case MAX31865_FAULT_VOLTAGE:
            puts("Fault: Under or over voltage");
            break;
        case MAX31865_FAULT_RTD_HIGH:
            puts("Fault: Temperature too high");
            break;
        case MAX31865_FAULT_RTD_LOW:
            puts("Fault: Temperature too low");
            break;
        default:
            puts("Fault: Unknown");
            break;
    }
}
