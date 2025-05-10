#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

class ActuatorEffectivenessCCServo : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessCCServo(ModuleParams *parent);
	virtual ~ActuatorEffectivenessCCServo() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

	const char *name() const override { return "CCServo"; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	float yaw_sp = 0.f;

protected:
	ActuatorVector _tilt_offsets;
	ActuatorEffectivenessRotors _cc_rotors;
	ActuatorEffectivenessTilts _cc_tilts;
	int _first_tilt_idx{0};

	struct YawTiltSaturationFlags {
		bool tilt_yaw_pos;
		bool tilt_yaw_neg;
	};

	YawTiltSaturationFlags _yaw_tilt_saturation_flags{};
};
