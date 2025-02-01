PARAM_DEFINE_FLOAT(CPC_Z_ACC_MAX_UP, 4.f);	// m/s^2
PARAM_DEFINE_FLOAT(CPC_Z_ACC_MAX_DOWN, 3.f);	// m/s^2

/**
 * 高度参考模式
 *
 * 设置为 0 以相对于地球坐标系原点控制高度。该原点在飞行过程中可能由于传感器漂移而上下移动。
 * 设置为 1 以相对于估计的地面距离控制高度。车辆将随着地形高度变化而上下移动。需要一个测距传感器。如果地面距离估计无效（如 `local_position.distance_bottom_valid` 消息为假），高度控制器将恢复为使用相对于原点的高度。
 * 设置为 2 以在静止时相对于地面控制高度（需要测距传感器），在水平移动时相对于地球坐标系原点控制高度。
 * 水平速度阈值由 `CPC_HOLD_MAX_XY` 参数控制。
 *
 * @最小值 0
 * @最大值 2
 * @取值 0 高度跟随
 * @取值 1 地形跟随
 * @取值 2 地形保持
 */
PARAM_DEFINE_INT32(CPC_ALT_MODE, 2);

/**
 * 位置保持启用的最大水平速度（设置为 0 禁用检查）
 *
 * 仅在 `CPC_POS_MODE` 为 0 或 `CPC_ALT_MODE` 为 2 时使用
 *
 * @单位 m/s
 * @最小值 0
 * @最大值 3
 * @小数位 2
 */
PARAM_DEFINE_FLOAT(CPC_XY_HOLD_MAX, 0.8f);

/**
 * 位置保持启用的最大垂直速度（设置为 0 禁用检查）
 *
 * 仅在 `CPC_ALT_MODE` 为 1 时使用
 *
 * @单位 m/s
 * @最小值 0
 * @最大值 3
 * @小数位 2
 */
PARAM_DEFINE_FLOAT(CPC_Z_HOLD_MAX, 0.6f);
