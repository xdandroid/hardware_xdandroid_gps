#include <hardware/gps.h>

#include <stdlib.h>

#define LOG_TAG "gps_xdandroid"
#include <utils/Log.h>

#define  GPS_DEBUG  1

#if GPS_DEBUG
#  define  D(...)   LOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

extern const GpsInterface* gps_get_hardware_interface();

const GpsInterface* gps__get_gps_interface(struct gps_device_t* dev)
{
    D("gps_get_hardware_interface");
    return gps_get_hardware_interface();
}

static int open_gps(const struct hw_module_t* module, char const* name,
        struct hw_device_t** device)
{
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    dev->get_gps_interface = gps__get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}

static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};

const struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "MSM7k GPS Module",
    .author = "XDAndroid",
    .methods = &gps_module_methods,
};
