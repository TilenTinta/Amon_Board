#include "gnss.h"
#include <stdio.h>

static int gnss_initialized = 0;

void gnss_init(void) {
    gnss_initialized = 1;
    printf("GNSS module initialized\n");
}

void gnss_update(GnssData *data) {
    if (!gnss_initialized) return;
    
    // Simulate GNSS data
    data->latitude = 37.7749;
    data->longitude = -122.4194;
    data->altitude = 100.5;
    data->speed = 25.3;
    data->satellites = 8;
    
    printf("GNSS updated: Lat=%.4f, Lon=%.4f\n", 
           data->latitude, data->longitude);
}

void gnss_cleanup(void) {
    gnss_initialized = 0;
    printf("GNSS module cleaned up\n");
}