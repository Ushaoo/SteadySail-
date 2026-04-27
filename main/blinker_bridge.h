#ifndef BLINKER_BRIDGE_H
#define BLINKER_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

// 启动 Blinker 桥接：注册控件 + 启动数据上报任务（1Hz）
// 内部会调用 blinker_init()。需在 control_params_init() 之后调用。
void blinker_bridge_start(void);

#ifdef __cplusplus
}
#endif

#endif // BLINKER_BRIDGE_H
