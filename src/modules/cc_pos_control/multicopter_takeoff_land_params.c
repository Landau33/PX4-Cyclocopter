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
 * @分组 多旋翼位置控制
 */
PARAM_DEFINE_FLOAT(MPC_TKO_RAMP_T, 3.f);

/**
 * 起飞爬升速度
 *
 * @单位 米/秒 (m/s)
 * @最小值 1
 * @最大值 5
 * @小数位 2
 * @分组 多旋翼位置控制
 */
PARAM_DEFINE_FLOAT(MPC_TKO_SPEED, 1.5f);

/**
 * 慢速降落第一阶段的高度
 *
 * 低于此高度时，下降速度将被限制在 "MPC_Z_VEL_MAX_DN"（或 "MPC_Z_V_AUTO_DN"）和 "MPC_LAND_SPEED" 之间的一个值。
 * 此值需要高于 "MPC_LAND_ALT2"。
 *
 * @单位 米 (m)
 * @最小值 0
 * @最大值 122
 * @小数位 1
 * @分组 多旋翼位置控制
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT1, 10.f);

/**
 * 慢速降落第二阶段的高度
 *
 * 低于此高度时，下降速度将被限制为 "MPC_LAND_SPEED"。
 * 此值需要低于 "MPC_LAND_ALT1"。
 *
 * @单位 米 (m)
 * @最小值 0
 * @最大值 122
 * @小数位 1
 * @分组 多旋翼位置控制
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT2, 5.f);

/**
 * 慢速降落第三阶段的高度
 *
 * 低于此高度时，如果存在激光雷达 (LIDAR)，下降速度将被限制为 "MPC_LAND_CRWL"。
 * 如果没有激光雷达，则此参数无效。
 *
 * @单位 米 (m)
 * @最小值 0
 * @最大值 122
 * @小数位 1
 * @分组 多旋翼位置控制
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT3, 1.f);

/**
 * 着陆下降速度
 *
 * @单位 米/秒 (m/s)
 * @最小值 0.6
 * @小数位 1
 * @分组 多旋翼位置控制
 */
PARAM_DEFINE_FLOAT(MPC_LAND_SPEED, 0.7f);

/**
 * 着陆爬行下降速度
 *
 * 在低于 MPC_LAND_ALT3 且有距离传感器数据可用时使用。
 *
 * @单位 米/秒 (m/s)
 * @最小值 0.1
 * @小数位 1
 * @分组 多旋翼位置控制
 */
PARAM_DEFINE_FLOAT(MPC_LAND_CRWL, 0.3f);
