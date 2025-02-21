/**
 * 滚转速率 P 增益
 *
 * 滚转速率比例增益，即角速度误差为 1 rad/s 时的控制输出。
 *
 * @最小值 0.01
 * @最大值 0.5
 * @小数位 3
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_ROLL_RATE_P, 0.15f);

/**
 * 滚转速率 I 增益
 *
 * 滚转速率积分增益。可以设置以补偿静态推力差异或重心偏移。
 *
 * @最小值 0.0
 * @小数位 3
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_ROLL_RATE_I, 0.2f);

/**
 * 滚转速率积分器限制
 *
 * 滚转速率积分器限制。可以设置以增加可用积分器的数量来对抗干扰，或者减少以改善大滚转力矩调整后的稳定时间。
 *
 * @最小值 0.0
 * @小数位 2
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_RR_INT_LIM, 0.30f);

/**
 * 滚转速率 D 增益
 *
 * 滚转速率微分增益。小值有助于减少快速振荡。如果值过大，振荡将再次出现。
 *
 * @最小值 0.0
 * @最大值 0.01
 * @小数位 4
 * @增量 0.0005
 */
PARAM_DEFINE_FLOAT(CC_ROLL_RATE_D, 0.003f);

/**
 * 滚转速率前馈
 *
 * 改善跟踪性能。
 *
 * @最小值 0.0
 * @小数位 4
 */
PARAM_DEFINE_FLOAT(CC_ROLL_RATE_FF, 0.0f);

/**
 * 滚转速率控制器增益
 *
 * 控制器的全局增益。
 *
 * 此增益缩放控制器的比例、积分和微分项：
 * output = CC_ROLL_RATE_K * (CC_ROLL_RATE_P * error
 *                          + CC_ROLL_RATE_I * error_integral
 *                          + CC_ROLL_RATE_D * error_derivative)
 * 设置 CC_ROLL_RATE_P=1 以实现理想的 PID 形式。
 * 设置 CC_ROLL_RATE_K=1 以实现并行形式的 PID。
 *
 * @最小值 0.01
 * @最大值 5.0
 * @小数位 4
 * @增量 0.0005
 */
PARAM_DEFINE_FLOAT(CC_ROLL_RATE_K, 1.0f);

/**
 * 俯仰速率 P 增益
 *
 * 俯仰速率比例增益，即角速度误差为 1 rad/s 时的控制输出。
 *
 * @最小值 0.01
 * @最大值 0.6
 * @小数位 3
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_PITCH_RATE_P, 0.15f);

/**
 * 俯仰速率 I 增益
 *
 * 俯仰速率积分增益。可以设置以补偿静态推力差异或重心偏移。
 *
 * @最小值 0.0
 * @小数位 3
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_PITCH_RATE_I, 0.2f);

/**
 * 俯仰速率积分器限制
 *
 * 俯仰速率积分器限制。可以设置以增加可用积分器的数量来对抗干扰，或者减少以改善大俯仰力矩调整后的稳定时间。
 *
 * @最小值 0.0
 * @小数位 2
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_PR_INT_LIM, 0.30f);

/**
 * 俯仰速率 D 增益
 *
 * 俯仰速率微分增益。小值有助于减少快速振荡。如果值过大，振荡将再次出现。
 *
 * @最小值 0.0
 * @小数位 4
 * @增量 0.0005
 */
PARAM_DEFINE_FLOAT(CC_PITCH_RATE_D, 0.003f);

/**
 * 俯仰速率前馈
 *
 * 改善跟踪性能。
 *
 * @最小值 0.0
 * @小数位 4
 */
PARAM_DEFINE_FLOAT(CC_PITCH_RATE_FF, 0.0f);

/**
 * 俯仰速率控制器增益
 *
 * 控制器的全局增益。
 *
 * 此增益缩放控制器的比例、积分和微分项：
 * output = CC_PITCH_RATE_K * (CC_PITCH_RATE_P * error
 *                           + CC_PITCH_RATE_I * error_integral
 *                           + CC_PITCH_RATE_D * error_derivative)
 * 设置 CC_PITCH_RATE_P=1 以实现理想的 PID 形式。
 * 设置 CC_PITCH_RATE_K=1 以实现并行形式的 PID。
 *
 * @最小值 0.01
 * @最大值 5.0
 * @小数位 4
 * @增量 0.0005
 */
PARAM_DEFINE_FLOAT(CC_PITCH_RATE_K, 1.0f);

/**
 * 偏航速率 P 增益
 *
 * 偏航速率比例增益，即角速度误差为 1 rad/s 时的控制输出。
 *
 * @最小值 0.0
 * @最大值 0.6
 * @小数位 2
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_YAW_RATE_P, 0.2f);

/**
 * 偏航速率 I 增益
 *
 * 偏航速率积分增益。可以设置以补偿静态推力差异或重心偏移。
 *
 * @最小值 0.0
 * @小数位 2
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_YAW_RATE_I, 0.1f);

/**
 * 偏航速率积分器限制
 *
 * 偏航速率积分器限制。可以设置以增加可用积分器的数量来对抗干扰，或者减少以改善大偏航力矩调整后的稳定时间。
 *
 * @最小值 0.0
 * @小数位 2
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_YR_INT_LIM, 0.30f);

/**
 * 偏航速率 D 增益
 *
 * 偏航速率微分增益。小值有助于减少快速振荡。如果值过大，振荡将再次出现。
 *
 * @最小值 0.0
 * @小数位 2
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_YAW_RATE_D, 0.0f);

/**
 * 偏航速率前馈
 *
 * 改善跟踪性能。
 *
 * @最小值 0.0
 * @小数位 4
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CC_YAW_RATE_FF, 0.0f);

/**
 * 偏航速率控制器增益
 *
 * 控制器的全局增益。
 *
 * 此增益缩放控制器的比例、积分和微分项：
 * output = CC_YAW_RATE_K * (CC_YAW_RATE_P * error
 *                         + CC_YAW_RATE_I * error_integral
 *                         + CC_YAW_RATE_D * error_derivative)
 * 设置 CC_YAW_RATE_P=1 以实现理想的 PID 形式。
 * 设置 CC_YAW_RATE_K=1 以实现并行形式的 PID。
 *
 * @最小值 0.0
 * @最大值 5.0
 * @小数位 4
 * @增量 0.0005
 */
PARAM_DEFINE_FLOAT(CC_YAW_RATE_K, 1.0f);

/**
 * 电池功率水平缩放
 *
 * 这通过尝试在电池工作范围内标准化性能来补偿电池电压随时间的下降。多旋翼应始终表现得像完全充电一样，但在较低的电池百分比下具有较低的最大加速度。
 * 例如，如果悬停是在 100% 电池电量下的 0.5 油门，在 60% 电池电量下它仍然是 0.5 油门。
 *
 * @布尔值
 */
PARAM_DEFINE_INT32(CC_BAT_SCALE_EN, 0);
