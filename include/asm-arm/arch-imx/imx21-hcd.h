#ifndef __IMX21_HCD_H__
#define __IMX21_HCD_H__
struct imx21_usb_platform_data {
    void (*set_mode)(int);
    void (*set_speed)(int);
    void (*set_suspend)(int);
    void (*set_oe)(int);
};

#endif
