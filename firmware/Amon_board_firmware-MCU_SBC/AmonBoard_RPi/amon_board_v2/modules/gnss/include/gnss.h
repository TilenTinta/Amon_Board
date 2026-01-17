#ifndef GNSS_H
#define GNSS_H

typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    int satellites;
} GnssData;

void gnss_init(void);
void gnss_update(GnssData *data);
void gnss_cleanup(void);

#endif