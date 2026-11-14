/*
 * Creates our flow sensors: PX4's OpticalFlowSystem (BSD-3-Clause, PX4 Development Team), for
 * sensors of type perch_optical_flow only, so it runs beside PX4's without either taking the
 * other's sensors. Load it from the model:
 *   <plugin filename="PerchOpticalFlowSystem" name="perch_sim::OpticalFlowSystem">
 *     <shift_only>true</shift_only>   <!-- false: PX4's flow, for comparison -->
 *   </plugin>
 */
#include <memory>
#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <sdf/Sensor.hh>

#include "OpticalFlowSensor.hpp"

namespace perch_sim
{

class OpticalFlowSystem :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate,
	public gz::sim::ISystemPostUpdate
{
public:
	void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &, gz::sim::EventManager &) final
	{
		_shift_only = _sdf->Get<bool>("shift_only", true).first;
	}

	void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) final
	{
		auto add = [&](const gz::sim::Entity & _entity,
			       const gz::sim::components::CustomSensor * _custom,
		const gz::sim::components::ParentEntity * _parent)->bool {
			sdf::Sensor data = _custom->Data();

			if (gz::sensors::customType(data) != kSensorType || _sensors.count(_entity))
			{
				return true; // someone else's custom sensor, or one we have
			}

			auto name = gz::sim::removeParentScope(gz::sim::scopedName(_entity, _ecm, "::", false), "::");
			data.SetName(name);

			if (data.Topic().empty())
			{
				data.SetTopic(gz::sim::scopedName(_entity, _ecm) + "/optical_flow");
			}

			auto sensor = std::make_unique<OpticalFlowSensor>(_shift_only);

			if (!sensor->Load(data) || !sensor->Init()) // as gz::sensors::SensorFactory
			{
				gzerr << "Failed to create optical flow sensor [" << name << "]" << std::endl;
				return true;
			}

			sensor->SetParent(_ecm.Component<gz::sim::components::Name>(_parent->Data())->Data());
			_ecm.CreateComponent(_entity, gz::sim::components::SensorTopic(sensor->Topic()));
			_sensors.emplace(_entity, std::move(sensor));
			return true;
		};

		// Loaded with the model, after Gazebo created its sensors: take those that exist on the
		// first update (PX4's version is a world plugin and only needs the new ones)
		if (_first_update) {
			_ecm.Each<gz::sim::components::CustomSensor, gz::sim::components::ParentEntity>(add);
			_first_update = false;
		}

		_ecm.EachNew<gz::sim::components::CustomSensor, gz::sim::components::ParentEntity>(add);
	}

	void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) final
	{
		if (!_info.paused) {
			for (auto &[entity, sensor] : _sensors) {
				sensor->Update(_info.simTime);
			}
		}

		_ecm.EachRemoved<gz::sim::components::CustomSensor>(
			[&](const gz::sim::Entity & _entity, const gz::sim::components::CustomSensor *)->bool {
			_sensors.erase(_entity);
			return true;
		});
	}

private:
	bool _shift_only {true};
	bool _first_update {true};
	std::unordered_map<gz::sim::Entity, std::unique_ptr<OpticalFlowSensor>> _sensors;
};

} // namespace perch_sim

GZ_ADD_PLUGIN(perch_sim::OpticalFlowSystem, gz::sim::System,
	      perch_sim::OpticalFlowSystem::ISystemConfigure,
	      perch_sim::OpticalFlowSystem::ISystemPreUpdate,
	      perch_sim::OpticalFlowSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(perch_sim::OpticalFlowSystem, "perch_sim::OpticalFlowSystem")
