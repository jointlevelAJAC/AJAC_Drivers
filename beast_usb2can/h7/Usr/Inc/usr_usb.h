#ifndef USB2CAN_USB_H_
#define USB2CAN_USB_H_

#include "usbd_custom_hid_if.h"
#include "usr_spi.h"

#define USB_CMD_SIZE 376
#define USB_DATA_SIZE 376
#define USB_DATA_CHK_SIZE 93

typedef struct _USB_CMD_Pack {
  float p_cmd;
  float v_cmd;
  float kp;
  float kd;
  float t_ff;
} USB_CMD_PACK_T;

// 124bytes two legs
typedef struct _USB_CMD {
  USB_CMD_PACK_T cmd_pack[6];
  uint32_t chip_flag[1];
} USB_CMD_TypeDef;

typedef struct _USB_DATA_Pack {
  float p_data;
  float v_data;
  float t_data;
  float uq;
  float ud;
} USB_DATA_PACK_T;

typedef struct _USB_DATA {
  USB_DATA_PACK_T data_pack[6];
  uint32_t chip_flag[1];
} USB_DATA_TypeDef;

typedef struct _USB_CMD_PACK {
  USB_CMD_TypeDef usb_cmd[CHIP_NUMBERS];
  uint32_t checksum;
} USB_CMD_PACK;

typedef struct _USB_DATA_PACK {
  USB_DATA_TypeDef usb_data[CHIP_NUMBERS];
  uint32_t checksum;
} USB_DATA_PACK;

typedef union _USB_CMD_U {
  USB_CMD_PACK usb_cmd_pack;
  uint8_t usb_cmd_pack_buf[USB_CMD_SIZE];
} USB_CMD_U;

typedef union _USB_DATA_U {
  USB_DATA_PACK usb_data_pack;
  uint8_t usb_data_pack_buf[USB_DATA_SIZE];
} USB_DATA_U;

typedef struct _USB_PROTOCAL {
  USB_CMD_U usb_cmd_u;
  USB_DATA_U usb_data_u;
  void (*convert_usb_cmd)(void);
  void (*convert_usb_data)(void);
} USB_PROTOCAL_TypeDef;

USB_PROTOCAL_TypeDef* get_usb_handle(void);
void usb_handle_init(void);

#endif