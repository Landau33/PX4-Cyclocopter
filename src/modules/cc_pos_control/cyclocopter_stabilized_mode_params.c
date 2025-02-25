/**
 * 自稳模式或高度模式下的最大倾斜角
 *
 * @单位 度 (deg)
 * @最小值 1
 * @最大值 70
 * @小数位 0
 * @增量 1
 */
PARAM_DEFINE_FLOAT(CPC_MAN_TILT_MAX, 35.f);

/**
 * 稳定模式、高度模式、位置模式下的最大手动偏航速率
 *
 * @单位 度/秒 (deg/s)
 * @最小值 0
 * @最大值 400
 * @小数位 0
 * @增量 10
 */
PARAM_DEFINE_FLOAT(CPC_YAW_RATE_MAX, 150.f);

/**
 * 稳定模式下的最小总推力
 *
 * 该值映射到稳定模式下油门操纵杆的最低位置。
 *
 * 推力过低会导致失去滚转、俯仰和偏航扭矩控制能力。
 * 使用空气模式（参见 MC_AIRMODE）可以在零推力时保持扭矩控制能力。
 *
 * @单位 规范化 (norm)
 * @最小值 0
 * @最大值 1
 * @小数位 2
 * @增量 0.01
 */
PARAM_DEFINE_FLOAT(CPC_MAN_THR_MIN, 0.08f);

/**
 * 稳定模式下的推力曲线映射
 *
 * 此参数定义了在稳定模式下，油门操纵杆输入如何映射到总推力。
 *
 * 如果使用默认设置（'重新缩放至悬停推力'），则操纵杆输入将线性重新缩放，使得居中的操纵杆对应于悬停推力（参见 CPC_THR_HOVER）。
 *
 * 选择 '不重新缩放' 可直接将操纵杆 1:1 映射到输出。这在悬停推力非常低且默认设置会导致过多失真的情况下很有用
 * （例如，如果悬停推力设置为 20%，则上部推力范围的 80% 被压缩到操纵杆范围的上半部分）。
 *
 * 注意：如果 CPC_THR_HOVER 设置为 50%，则模式 0 和 1 是相同的。
 *
 * @取值 0 重新缩放至悬停推力
 * @取值 1 不重新缩放
 */
PARAM_DEFINE_INT32(CPC_THR_CURVE, 0);
