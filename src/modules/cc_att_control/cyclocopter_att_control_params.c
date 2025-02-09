PARAM_DEFINE_FLOAT(CC_ROLL_P, 6.5f);
PARAM_DEFINE_FLOAT(CC_PITCH_P, 6.5f);
PARAM_DEFINE_FLOAT(CC_YAW_P, 2.8f);

/**
 * 偏航权重
 *
 * 一个介于 [0,1] 的分数，用于在非线性姿态控制中降低偏航相对于横滚和俯仰的优先级。
 * 降低偏航的优先级是必要的，因为多旋翼飞行器在偏航轴上的控制能力远低于其他轴，
 * 并且从稳定悬停或三维导航的角度来看，偏航并不是关键因素。
 *
 * 对于偏航控制的调整，请使用 CC_YAW_P。此比例对偏航增益没有影响。
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 */

 // 滚翼是否需要有待验证
PARAM_DEFINE_FLOAT(CC_YAW_WEIGHT, 0.4f);

/**
 * 最大横滚速率
 *
 * 限制手动和自动模式（除acro模式外）下的横滚速率。
 * 在自主模式下，对于大角度旋转，此参数有助于避免过大的控制输出和混控饱和。
 *
 * 该参数不仅受限于飞行器的特性，还受限于陀螺仪的最大测量速率。
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 */
PARAM_DEFINE_FLOAT(CC_ROLLRATE_MAX, 220.0f);

/**
 * 最大俯仰速率
 *
 * 限制手动和自动模式（除acro模式外）下的俯仰速率。
 * 在自主模式下，对于大角度旋转，此参数有助于避免过大的控制输出和混控饱和。
 *
 * 该参数不仅受限于飞行器的特性，还受限于陀螺仪的最大测量速率。
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * 最大偏航速率
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 */
PARAM_DEFINE_FLOAT(CPC_YAW_RATE_MAX, 200.0f);

/**
 * 手动倾斜输入滤波时间常数
 *
 * 将此参数设置为0会禁用滤波器
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(C_MAN_TILT_TAU, 0.0f);
