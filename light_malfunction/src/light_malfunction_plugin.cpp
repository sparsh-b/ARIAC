#include <light_malfunction_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <algorithm>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(LightPlugin)


LightSettingPrivate::LightSettingPrivate(int num_intensities): flashing(true), range(0) {
    this->currentBlockIndex = rand() % num_intensities;
}

    /// \brief Find the link holding the light to control.
    /// If multiple models are nested, this function is recursively called
    /// until the link is found.
    /// \param[in] _model A model to check.
    /// \param[in] _lightName the name of the light.
    /// \param[in] _linkName the name of the link.
    /// \return A pointer to the link. If not found, nullptr is returned.
physics::LinkPtr LightSettingPrivate::FindLinkForLight(const physics::ModelPtr &_model, const std::string &_lightName,
                                          const std::string &_linkName) {
            auto childLink = _model->GetChildLink(_linkName);
            if (childLink && childLink->GetSDF()->HasElement("light"))
            {
                auto sdfLight = childLink->GetSDF()->GetElement("light");
                while (sdfLight)
                {
                if (sdfLight->Get<std::string>("name") == _lightName)
                {
                    return childLink;
                }
                sdfLight = sdfLight->GetNextElement("light");
                }
            }
            for (auto model: _model->NestedModels())
            {
                auto foundLink = this->FindLinkForLight(model, _lightName, _linkName);
                if (foundLink)
                {
                return foundLink;
                }
            }

            return nullptr;
}


LightSetting::LightSetting(std::string &lightId, bool is_chlng_lyt, const LightPluginPrivate *lpp,
                            //const sdf::ElementPtr &_sdf, 
                            const physics::ModelPtr &_model, 
                            const common::Time &_currentTime) {
                            
    this->dataPtr = new LightSettingPrivate(lpp->intensities.size());
    
    int posDelim = lightId.rfind("/");
    this->dataPtr->name = lightId.substr(posDelim+1, lightId.length());
    this->dataPtr->startTime = _currentTime;
    this->dataPtr->range = lpp->light_range;

    // link which holds this light
    this->dataPtr->link = this->dataPtr->FindLinkForLight(_model, this->dataPtr->name, lightId.substr(0, posDelim));

    if (is_chlng_lyt) {
        for (const float &intensity : lpp->intensities) {
            auto color = ignition::math::Color(intensity, intensity, intensity);
            auto block = std::make_shared<Block>(lpp->intensity_duration, 0, color);
            this->dataPtr->blocks.push_back(block);
        }
    }
    // If the current light is not a challenge light, shine it in full brightness for the entire duration of the challenge
    else {
        auto color = ignition::math::Color(lpp->nominal_brightness, lpp->nominal_brightness, lpp->nominal_brightness);
        auto block = std::make_shared<Block>(lpp->challenge_duration, 0, color);
        this->dataPtr->blocks.push_back(block);
    }    
};

LightSetting::~LightSetting() {}

void LightSetting::InitPubLight(const transport::PublisherPtr &_pubLight) {
    
    // The PublisherPtr
    this->dataPtr->pubLight = _pubLight;

    // Make a message
    this->dataPtr->msg.set_name(this->dataPtr->link->GetScopedName() + "::" + this->dataPtr->name);
    printf("%s\n", this->dataPtr->link->GetScopedName().c_str());
    this->dataPtr->msg.set_range(this->dataPtr->range);
}


LightPlugin::LightPlugin() : ModelPlugin(), dataPtr(new LightPluginPrivate) {
    
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init();

    // advertise the topic to update lights
    this->dataPtr->pubLight = this->dataPtr->node->Advertise<gazebo::msgs::Light>("~/light/modify");
    this->dataPtr->pubLight->WaitForConnection();

    // TODO: Should be read from config file finally
    // append link name to light names read from config file
    this->dataPtr->all_light_ids = {"lights_link/conveyor_light_1",
                                    "lights_link/bin_light_1",
                                    "lights_link/bin_light_2", 
                                    "lights_link/assembly_light_1",
                                    "lights_link/assembly_light_2",
                                    "lights_link/assembly_light_3",
                                    "lights_link/assembly_light_4"};

    this->dataPtr->chlg_light_ids = {"lights_link/conveyor_light_1",
                                     "lights_link/bin_light_1",
                                     "lights_link/bin_light_2"};

    this->dataPtr->scenario = "flicker";
    this->dataPtr->challenge_duration = 5;

    this->dataPtr->light_range = 1000;
    if (this->dataPtr->scenario == "flicker") {
        this->dataPtr->intensities = {0.23, 0.65, 0.2, 0.5, 0.25, 0.29, 0.63, 0.21, 0.19, 0.29, 0.74, 0.32, 0.2, 0.39, 0.23, 0.45, 0.21, 0.5, 0.28, 0.81, 0.32, 0.21, 0.61, 0.22, 0.84, 0.29, 0.51, 0.19, 0.67, 0.24, 0.67, 0.22, 0.58, 0.27, 0.44, 0.56, 0.28, 0.6, 0.21, 0.48, 0.43, 0.37, 0.37, 0.18, 0.76, 0.27, 0.28, 0.23, 0.9, 0.32, 0.32, 0.34, 0.55, 0.55, 0.86, 0.31, 0.43, 0.81, 0.33, 0.33, 0.22, 0.8, 0.27, 0.75, 0.31, 0.19, 0.59, 0.22, 0.59, 0.41, 0.25, 0.88, 0.29, 0.41, 0.42, 0.8, 0.48, 0.33, 0.82, 0.31, 0.39, 0.5, 0.36, 0.52, 0.86, 0.47, 0.51, 0.87, 0.4, 0.96, 0.96, 0.22, 0.63, 0.2, 0.61, 0.3, 0.83, 0.31, 0.33, 0.33, 0.6, 0.38, 0.68, 0.29, 0.59, 0.35, 0.53, 0.44, 0.25, 0.81, 0.32, 0.41, 0.57, 0.28, 1.0, 0.39, 0.32, 0.72, 0.28, 0.77, 0.38, 0.53, 0.7, 0.34, 0.59, 0.23, 0.54, 0.22, 0.55, 0.21, 0.54, 0.21, 0.61, 0.23, 0.52, 0.37, 0.21, 0.62, 0.33, 0.46, 0.22, 0.51, 0.67, 0.41, 0.23, 0.56, 0.24, 0.75, 0.28, 0.59, 0.35, 0.43, 0.53, 0.32, 0.96, 0.42, 0.46, 0.57, 0.4, 0.62, 0.38, 0.68, 0.38, 0.9, 0.42, 0.94, 0.4, 0.36, 0.58, 0.67, 0.6, 0.36, 0.78, 0.49, 0.5, 0.64, 0.53, 0.35, 0.92, 0.38, 0.45, 0.45, 0.81, 0.5, 0.37, 0.71, 0.43, 0.56, 0.86, 0.37, 0.34, 0.4, 0.75, 0.39, 0.26, 0.72, 0.27, 0.96, 0.35, 0.81, 0.3, 0.51, 0.24, 0.83, 0.3, 0.2, 0.46, 0.52, 0.26, 0.61, 0.17, 0.56, 0.15, 0.15, 0.56, 0.3, 0.31, 0.15, 0.48, 0.26, 0.27, 0.48, 0.19, 0.57, 0.1, 0.34, 0.44, 0.36, 0.18, 0.62, 0.19, 0.23, 0.31, 0.31, 0.24, 0.3, 0.44, 0.19, 0.78, 0.3, 0.79, 0.31, 0.2, 0.45, 0.58, 0.35};
        this->dataPtr->intensity_duration = 0.05;
    } else if (this->dataPtr->scenario == "dim") {
        this->dataPtr->intensities = {0.2};
        this->dataPtr->intensity_duration = this->dataPtr->challenge_duration;
    }
}


LightPlugin::~LightPlugin() {}


void LightPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    
    // Store the pointers to the model and world
    this->dataPtr->model = _parent;
    this->dataPtr->world = _parent->GetWorld();
    if (_sdf->HasElement("nominal_brightness")) {
        this->dataPtr->nominal_brightness = _sdf->Get<double>("nominal_brightness") / 100.0;
    }

    // Get the current time
    common::Time currentTime = this->dataPtr->world->SimTime();

    for (std::string &light : this->dataPtr->all_light_ids) {
        bool is_chlng_lyt = std::find(this->dataPtr->chlg_light_ids.begin(), this->dataPtr->chlg_light_ids.end(), light)
                          != this->dataPtr->chlg_light_ids.end();
        auto setting = std::make_shared<LightSetting>(light, is_chlng_lyt, this->dataPtr.get(), this->dataPtr->model,
                                                           currentTime);
        setting->InitPubLight(this->dataPtr->pubLight);
        this->dataPtr->listLight.push_back(setting);
    }
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LightPlugin::OnUpdate, this));
}


void LightPlugin::OnUpdate() {
    
    common::Time currentTime = this->dataPtr->world->SimTime();

    for (auto &setting : this->dataPtr->listLight) {
        setting->UpdateLightInEnv(currentTime);
    }
}


void LightSetting::UpdateLightInEnv(const common::Time &_currentTime) {
    
    int index = this->dataPtr->currentBlockIndex;

    if (_currentTime <  this->dataPtr->startTime ||
        _currentTime >= this->dataPtr->startTime + this->dataPtr->blocks[index]->duration + 
                        this->dataPtr->blocks[index]->interval)
    {
        // initialize the start time.
        this->dataPtr->startTime = _currentTime;
        // proceed to the next block.
        index++;
        if (index >= static_cast<int>(this->dataPtr->blocks.size())) {
            index = 0;
        }
        this->dataPtr->currentBlockIndex = index;
    }

    //Time to flash
    if (_currentTime - this->dataPtr->startTime <= this->dataPtr->blocks[index]->duration) {
        // If the current block started execution only at the current timestep, call ()
        if (this->dataPtr->startTime == _currentTime) {
            this->Flash();
        }
        // otherwise, call Flash() only if the light is not lit yet.
        else if (!this->dataPtr->flashing) {
            this->Flash();
        }
    }
    // Time to dim
    else {
        if (this->dataPtr->flashing) {
            this->Dim();
        }
    }
}


void LightSetting::Flash() {
    // Set the range to the default value.
    this->dataPtr->msg.set_range(this->dataPtr->range);
    // set the color of light.
    if (this->dataPtr->blocks[this->dataPtr->currentBlockIndex]->color != ignition::math::Color::Black)
    {
        msgs::Set(this->dataPtr->msg.mutable_diffuse(),
        this->dataPtr->blocks[this->dataPtr->currentBlockIndex]->color);
        msgs::Set(this->dataPtr->msg.mutable_specular(),
        this->dataPtr->blocks[this->dataPtr->currentBlockIndex]->color);
    }
    // Send the message.
    this->dataPtr->pubLight->Publish(this->dataPtr->msg);
    // Update the state.
    this->dataPtr->flashing = true;
}


void LightSetting::Dim() {
    // Set the range to zero.
    this->dataPtr->msg.set_range(0.0);
    // Send the message.
    this->dataPtr->pubLight->Publish(this->dataPtr->msg);
    // Update the state.
    this->dataPtr->flashing = false;
}
