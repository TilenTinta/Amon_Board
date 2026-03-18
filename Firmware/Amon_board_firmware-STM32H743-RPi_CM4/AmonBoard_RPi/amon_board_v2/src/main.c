#include "main.h"
#include <stdio.h>

void app_run(void) {
    printf("Amon Lander starting...\n");
    
    // Initialize GNSS module
    gnss_init();
    
    // Get GNSS data
    GnssData gps_data;
    gnss_update(&gps_data);
    
    printf("Altitude: %.1fm, Speed: %.1fm/s, Satellites: %d\n",
           gps_data.altitude, gps_data.speed, gps_data.satellites);
    
    gnss_cleanup();
    printf("Amon Lander finished.\n");
}

int main() {
    app_run();
    return 0;
}