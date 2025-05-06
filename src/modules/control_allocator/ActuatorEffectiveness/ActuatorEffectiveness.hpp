#pragma once

#include <cstdint>

#include <matrix/matrix/math.hpp>
#include <uORB/topics/control_allocator_status.h>

enum class AllocationMethod {
	NONE = -1,
	PSEUDO_INVERSE = 0,
	SEQUENTIAL_DESATURATION = 1,
	AUTO = 2,
};

enum class ActuatorType {
	MOTORS = 0,
	SERVOS,

	COUNT
};

enum class EffectivenessUpdateReason {
	NO_EXTERNAL_UPDATE = 0, // 无外部更新
	CONFIGURATION_UPDATE = 1, // 配置更新
	MOTOR_ACTIVATION_UPDATE = 2, // 电机激活更新
};

class ActuatorEffectiveness
{
public:
	ActuatorEffectiveness() = default;
	virtual ~ActuatorEffectiveness() = default;

	static constexpr int NUM_ACTUATORS = 16;
	static constexpr int NUM_AXES = 6;

	enum ControlAxis {
		ROLL = 0,
		PITCH,
		YAW,
		THRUST_X,
		THRUST_Y,
		THRUST_Z
	};

	static constexpr int MAX_NUM_MATRICES = 2;

	using EffectivenessMatrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>;
	using ActuatorVector = matrix::Vector<float, NUM_ACTUATORS>;

	enum class FlightPhase {
		HOVER_FLIGHT = 0, // 悬停飞行阶段
		FORWARD_FLIGHT = 1, // 前飞阶段
		TRANSITION_HF_TO_FF = 2, // 悬停到前飞过渡阶段
		TRANSITION_FF_TO_HF = 3 // 前飞到悬停过渡阶段
	};

	struct Configuration {
		/**
		 * 向选定的矩阵中添加一个效应器，返回索引；如果失败，返回-1
		 */
		int addActuator(ActuatorType type, const matrix::Vector3f &torque, const matrix::Vector3f &thrust);

		/**
		 * 在手动向选定矩阵添加N个效应器后调用此函数
		 */
		void actuatorsAdded(ActuatorType type, int count);

		int totalNumActuators() const;

		/// 已配置的效果矩阵。执行器按顺序填充，先电机后舵机
		EffectivenessMatrix effectiveness_matrices[MAX_NUM_MATRICES];

		int num_actuators_matrix[MAX_NUM_MATRICES]; ///< 当前数量，也是下一个要填充到效应矩阵中的效应器索引
		ActuatorVector trim[MAX_NUM_MATRICES];

		ActuatorVector linearization_point[MAX_NUM_MATRICES];

		int selected_matrix;

		uint8_t matrix_selection_indexes[NUM_ACTUATORS * MAX_NUM_MATRICES]; // 矩阵选择索引
		int num_actuators[(int)ActuatorType::COUNT]; // 不同类型效应器的数量
	};

	/**
	 * 设置当前飞行阶段
	 *
	 * @param 飞行阶段
	 */
	virtual void setFlightPhase(const FlightPhase &flight_phase)
	{
		_flight_phase = flight_phase;
	}

	/**
	 * 获取效应矩阵的数量。必须小于等于MAX_NUM_MATRICES。
	 * 这个值预计保持不变。
	 */
	virtual int numMatrices() const { return 1; }

	/**
	 * 如果配置为AUTO，则获取每个矩阵所需的分配方法
	 */
	virtual void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const
	{
		for (int i = 0; i < MAX_NUM_MATRICES; ++i) {
			allocation_method_out[i] = AllocationMethod::PSEUDO_INVERSE;
		}
	}

	/**
	 * 查询混合矩阵的横滚、俯仰和偏航列是否需要归一化
	 */
	virtual void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const
	{
		for (int i = 0; i < MAX_NUM_MATRICES; ++i) {
			normalize[i] = false;
		}
	}

	/**
	 * 如果已更新，获取控制效应矩阵
	 *
	 * @return 如果已更新并设置矩阵，则返回true
	 */
	virtual bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) { return false;}

	/**
	 * 获取当前飞行阶段
	 *
	 * @return 飞行阶段
	 */
	const FlightPhase &getFlightPhase() const
	{
		return _flight_phase;
	}

	/**
	* 显示名称
	*/
	virtual const char *name() const = 0;

	/**
	 * 来自控制分配的回调，允许操作设定点。
	 * 用于将辅助控制分配给效应器（例如襟翼和扰流板）。
	 *
	 * @param actuator_sp 输入和输出设定点
	 */
	virtual void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) {}

	/**
	 * 来自控制分配的回调，允许操作设定点。
	 * 可用于添加非线性或外部项。
	 * 它在矩阵乘法之后和最终裁剪之前调用。
	 * @param actuator_sp 输入和输出设定点
	 */
	virtual void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
				    int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
				    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) {}

	/**
	 * 获取需要停止的电机的位掩码
	 */
	virtual uint32_t getStoppedMotors() const { return _stopped_motors_mask; }

	/**
	 * 填充未分配的扭矩和推力，由效应类型定制。
	 * 可以为每种类型单独实现。如果没有实现，则使用效应矩阵代替。
	 */
	virtual void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) {}

	/**
	 * 停止由stoppable_motors_mask掩码标记且需求推力为零的电机
	 *
	 * @param stoppable_motors_mask 标记需要停止的电机的掩码
	 * @param actuator_sp 分配结果，用于确定是否需要停止电机
	 */
	virtual void stopMaskedMotorsWithZeroThrust(uint32_t stoppable_motors_mask, ActuatorVector &actuator_sp);

protected:
	FlightPhase _flight_phase{FlightPhase::HOVER_FLIGHT}; // 当前飞行阶段，默认为悬停飞行
	uint32_t _stopped_motors_mask{0}; // 停止的电机掩码
};
