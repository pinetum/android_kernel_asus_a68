#ifndef ___ICATCH7002A_H__
#define ___ICATCH7002A_H__

#include "msm_sensor.h"
#include "yuv_sensor.h"

#define WITH_INT // no define: polling mode, define: interrupt mode

#ifdef WITH_INT
irqreturn_t iCatch_irq_handler(int irq, void *data);
#endif

int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val);
int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val);
int icatch_i2c_debuginit(void);
int sensor_set_mode(int  type);
unsigned int get_fw_version_in_isp(void);
unsigned int get_fw_version_in_isp_fromISP(void);
void create_iCatch_proc_file(void);
void iCatch_init(void);
void iCatch_release_sensor(void);
void wait_for_AWB_ready(void);
void iCatch_enable_exif(bool enable, bool is_preExif);
    
void iCatch_start_AF(bool on, isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w);
uint16_t iCatch_get_AF_result(struct msm_sensor_ctrl_t *s_ctrl);
void iCatch_set_led_mode(led_mode_t mode);  //ASUS_BSP LiJen "[A68][13M][NA][Others]implement LED/Flash mode"
void iCatch_set_wb_mode(config3a_wb_t mode); //ASUS_BSP LiJen "[A68][13M][NA][Others]implement WB mode"
void iCatch_set_ev_mode(int16_t mode); //ASUS_BSP LiJen "[A68][13M][NA][Others]implement EV mode"
void iCatch_set_iso_mode(int16_t mode); //ASUS_BSP LiJen "[A68][13M][NA][Others]implement ISO mode"
void iCatch_set_flicker_mode(int16_t mode); //ASUS_BSP LiJen "[A68][13M][NA][Others]implement FLICKER mode"
void iCatch_set_acelock_mode(int16_t mode); //ASUS_BSP LiJen "[A68][13M][NA][Others]implement AECLOCK mode"
void iCatch_set_awblock_mode(int16_t mode);  //ASUS_BSP LiJen "[A68][13M][NA][Others]implement AWBLOCK mode"
void iCatch_set_caf_mode(bool continuous);  //ASUS_BSP LiJen "[A68][13M][NA][Others]implement CAF mode"
void iCatch_set_scene_mode(camera_bestshot_mode_type mode); //ASUS_BSP LiJen "[A68][13M][NA][Others]implement SCENE mode"
void iCatch_set_ultrapixel(int16_t mode);
void iCatch_set_effect_mode(int16_t mode); //ASUS_BSP LiJen "[A68][13M][NA][Others]implement EFFECT mode"
void iCatch_set_aura_value(int16_t mode); //ASUS_BSP LiJen "[A68][13M][NA][Others]implement AURA mode"
void iCatch_checkAFMode(void);
void iCatch_get_exif(struct exif_cfg *exif);	//ASUS_BSP Stimber "Implement EXIF info for camera with ISP"
bool iCatch_exif_flow_control(void);	//ASUS_BSP Stimber "Add for preview info control"
void iCatch_set_general_cmd(struct general_cmd_cfg *cmd);
void iCatch_get_general_cmd(struct general_cmd_cfg *cmd);
void create_iCatch_switch_file(void);
void remove_iCatch_switch_file(void);
void iCatch_enable_autonight(bool enable);

#endif //___ICATCH7002A_H__
