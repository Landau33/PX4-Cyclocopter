/**
 * 平滑起飞斜坡时间常数
 *
 * 增加此值将使爬升速率控制的起飞过程变慢。
 * 如果太慢，无人机可能会刮到地面并翻倒。
 * 时间常数为 0 将禁用斜坡。
 *
 * @单位 秒 (s)
 * @最小值 0
 * @最大值 5
 */
PARAM_DEFINE_FLOAT(CPC_TKOF_RAMP_T, 3.f);

/**
 * 起飞爬升速度
 *
 * @单位 米/秒 (m/s)
 * @最小值 1
 * @最大值 5
 * @小数位 2
 */
PARAM_DEFINE_FLOAT(CPC_TKOF_SPEED, 1.5f);

/**
 * 慢速降落第一阶段的高度
 *
 * 低于此高度时，下降速度将被限制在 "CPC_Z_VEL_MAX_DOWN" 和 "CPC_LAND_SPEED" 之间的一个值。
 * 此值需要高于 "CPC_LAND_ALT2"。
 *
 * @单位 米 (m)
 * @最小值 0
 * @最大值 122
 * @小数位 1
 */
PARAM_DEFINE_FLOAT(CPC_LAND_ALT1, 10.f);

/**
 * 慢速降落第二阶段的高度
 *
 * 低于此高度时，下降速度将被限制为 "CPC_LAND_SPEED"。
 * 此值需要低于 "CPC_LAND_ALT1"。
 *
 * @单位 米 (m)
 * @最小值 0
 * @最大值 122
 * @小数位 1
 */
PARAM_DEFINE_FLOAT(CPC_LAND_ALT2, 5.f);

/**
 * 着陆下降速度
 *
 * @单位 米/秒 (m/s)
 * @最小值 0.6
 * @小数位 1
 */
PARAM_DEFINE_FLOAT(CPC_LAND_SPEED, 0.7f);
