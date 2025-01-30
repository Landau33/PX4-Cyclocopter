#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace ControlMath
{
void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// 将推力向量转换为姿态设定点
	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
	att_sp.thrust_body[2] = -thr_sp.length(); // 设置机体坐标系下的 Z 轴推力大小
}

void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
	// 计算倾斜角度
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);

	// 限制倾斜角度
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	// 处理特殊情况：平行向量
	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	// 重新计算 body_unit 向量，确保其与世界坐标系的夹角不超过最大倾斜角度
	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// 如果输入向量接近零向量，则默认设置为 Z 轴正方向
	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	body_z.normalize();

	// 计算期望的 Yaw 方向在 XY 平面内的单位向量，并旋转 PI/2 角度
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

	// 计算期望的机体 X 轴，垂直于 body_z
	Vector3f body_x = y_C % body_z;

	// 当飞行器倒置时，保持机头向前
	if (body_z(2) < 0.f) {
		body_x = -body_x;
	}

	// 如果推力几乎在 XY 平面上，设置 X 轴向下以构造正确的旋转矩阵（但实际不会使用 Yaw 分量）
	if (fabsf(body_z(2)) < 0.000001f) {
		body_x.zero();
		body_x(2) = 1.f;
	}

	body_x.normalize();

	// 计算期望的机体 Y 轴
	const Vector3f body_y = body_z % body_x;

	Dcmf R_sp;

	// 填充旋转矩阵
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// 将四元数设定点复制到姿态设定点主题中
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// 计算欧拉角，仅用于记录日志，不得用于控制
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();
}

Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	// 如果 v0 和 v1 的和不超过最大值，则直接返回其和
	if (Vector2f(v0 + v1).norm() <= max) {
		return v0 + v1;

	} else if (v0.length() >= max) {
		// 如果 v0 的长度已经超过最大值，则返回 v0 的归一化向量乘以最大值
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		// 如果两个向量相等，则返回 v0 的归一化向量乘以最大值
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		// 如果第一个向量接近零，则返回 v1 的归一化向量乘以最大值
		return v1.normalized() * max;

	} else {
		// 使用二次方程求解缩放因子 s，使得最终向量的模长不超过最大值
		Vector2f u1 = v1.normalized();
		float m = u1.dot(v0);
		float c = v0.dot(v0) - max * max;
		float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

bool cross_sphere_line(const Vector3f &sphere_c, const float sphere_r,
		       const Vector3f &line_a, const Vector3f &line_b, Vector3f &res)
{
	// 将球心投影到线段上
	Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		// 球体与线段相交，计算交点
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.f) {
			// 目标航路点已经在后面
			res = line_b;

		} else {
			// 目标航路点在前面
			res = d + ab_norm * dx_len;
		}

		return true;

	} else {
		// 没有交点，返回最近点
		res = d;

		// 上一个航路点仍然在前面
		if ((sphere_c - line_a) * ab_norm < 0.f) {
			res = line_a;
		}

		// 目标航路点已经在后面
		if ((sphere_c - line_b) * ab_norm > 0.f) {
			res = line_b;
		}

		return false;
	}
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// 如果没有 NaN，将 addition 加到 setpoint 上
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// 如果 setpoint 是 NaN，取 addition 的值
		setpoint = addition;
	}

	// 如果 addition 或两者都是 NaN，不做任何操作
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// 添加零向量会将 NaN 元素替换为零
	addIfNotNanVector3f(vector, Vector3f());
}

} // ControlMath
