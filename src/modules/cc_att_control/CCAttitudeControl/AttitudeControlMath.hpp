#pragma once

#include <matrix/matrix/math.hpp>

namespace AttitudeControlMath
{
/**
 * 旋转一个倾斜四元数（不包含偏航旋转），使得当其被偏航设定值旋转后，
 * 所得到的倾斜效果与使用车辆当前偏航角度旋转后的效果相同。
 * @param q_sp_tilt 纯倾斜四元数（偏航 = 0），需要进行修正
 * @param q_att 当前车辆的姿态
 * @param q_sp_yaw 欲达到的纯偏航四元数设定值
 */
void inline correctTiltSetpointForYawError(matrix::Quatf &q_sp_tilt, const matrix::Quatf &q_att,
		const matrix::Quatf &q_sp_yaw)
{
	const matrix::Vector3f z_unit(0.f, 0.f, 1.f);

	// 从当前姿态中提取偏航角
	const matrix::Vector3f att_z = q_att.dcm_z();
	const matrix::Quatf q_tilt(z_unit, att_z);
	const matrix::Quatf q_yaw = q_tilt.inversed() * q_att; // 这不是欧拉偏航角

	// 寻找一个四元数，使其在被偏航设定值旋转后，与机体坐标系对齐
	// 为此，解方程 q_yaw * q_tilt_ne = q_sp_yaw * q_sp_rp_compensated
	const matrix::Quatf q_sp_rp_compensated = q_sp_yaw.inversed() * q_yaw * q_sp_tilt;

	// 提取修正后的倾斜角
	const matrix::Vector3f att_sp_z = q_sp_rp_compensated.dcm_z();
	q_sp_tilt = matrix::Quatf(z_unit, att_sp_z);
}
}
