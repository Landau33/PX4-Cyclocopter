#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

namespace ControlMath
{
/**
 * 将推力向量和偏航设定点转换为期望的姿态。
 * @param thr_sp 期望的三维推力向量
 * @param yaw_sp 期望的偏航角
 * @param att_sp 要填充的姿态设定点
 */
void thrustToAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

/**
 * 限制两个单位向量之间的倾斜角度。
 * @param body_unit 可调整的单位向量，如果角度过大将被调整
 * @param world_unit 固定的参考单位向量
 * @param max_angle 向量之间允许的最大倾斜角度（弧度）
 */
void limitTilt(matrix::Vector3f &body_unit, const matrix::Vector3f &world_unit, const float max_angle);

/**
 * 将机体 Z 轴向量和偏航设定点转换为期望的姿态。
 * @param body_z 世界坐标系中指向期望机体 Z 轴方向的三维向量
 * @param yaw_sp 期望的偏航设定点
 * @param att_sp 要填充的姿态设定点
 */
void bodyzToAttitude(matrix::Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

/**
 * 输出两个向量的和，但尊重限制和优先级。
 * v0 和 v1 的和受到约束，使得 v0 在最大可用模长下具有优先权。
 * 这意味着如果 (v0 + v1) 的长度超过最大值，则进行约束以确保 v0 的优先权。
 *
 * @param v0 具有优先权的二维向量，给定最大可用模长
 * @param v1 次要优先权的二维向量，给定最大可用模长
 * @param max 最大可用模长
 * @return 二维向量
 */
matrix::Vector2f constrainXY(const matrix::Vector2f &v0, const matrix::Vector2f &v1, const float &max);

/**
 * 此方法用于平滑两条线之间的拐角。
 *
 * @param sphere_c 球体中心
 * @param sphere_r 球体半径
 * @param line_a 线段起点
 * @param line_b 线段终点
 * @param res 结果向量
 * @return 布尔值
 *
 * 注意：此方法目前未在任何地方使用，并且在使用前需要审查。
 */
bool cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r, const matrix::Vector3f &line_a,
		       const matrix::Vector3f &line_b, matrix::Vector3f &res);

/**
 * 添加前馈到设定点，确保现有的或添加的 NaN 不影响控制。
 * 此函数有助于支持位置、速度、加速度等不同设定点组合，其中 NaN 表示未提交的值。
 * @param setpoint 可能包含 NaN 的现有设定点
 * @param addition 要添加的值/NAN
 */
void addIfNotNan(float &setpoint, const float addition);

/**
 * 对 Vector3f 的每个元素分别调用 addIfNotNan 函数。
 * @see addIfNotNan
 */
void addIfNotNanVector3f(matrix::Vector3f &setpoint, const matrix::Vector3f &addition);

/**
 * 将 Vector3f 中的 NaN 元素替换为零。
 * @param vector 可能包含 NaN 元素的向量
 */
void setZeroIfNanVector3f(matrix::Vector3f &vector);
}
