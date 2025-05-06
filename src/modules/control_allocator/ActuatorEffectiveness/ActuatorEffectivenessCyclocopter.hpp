#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

class ActuatorEffectivenessCyclocopter : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessCyclocopter(ModuleParams *parent);
	virtual ~ActuatorEffectivenessCyclocopter() = default;

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

	const char *name() const override { return "Cyclocopter"; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

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
