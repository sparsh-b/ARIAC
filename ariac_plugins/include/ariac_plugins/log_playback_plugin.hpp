#ifndef ARIAC_PLUGINS__LOG_PLAYBACK_PLUGIN_HPP_
#define ARIAC_PLUGINS__LOG_PLAYBACK_PLUGIN_HPP_

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <vector>
#include <map>

// For std::unique_ptr, could be removed
#include <memory>

namespace ariac_plugins
{
    /// Example ROS-powered Gazebo plugin with some useful boilerplate.
    /// \details This is a `ModelPlugin`, but it could be any supported Gazebo plugin type, such as
    /// System, Visual, GUI, World, Sensor, etc.
    class LogPlaybackPlugin : public gazebo::SystemPlugin
    {
    public:
        /// Destructor
        virtual ~LogPlaybackPlugin();
        void Load(int _argc, char **_argv);

    private:
        /// Recommended PIMPL pattern. This variable should hold all private
        /// data members.
        void Init();
        void Update();
        void OnWorldCreated();
        gazebo::event::ConnectionPtr updateConn;
        gazebo::event::ConnectionPtr worldCreatedConn;
        gazebo::transport::NodePtr node;
        gazebo::transport::PublisherPtr droneTogglePub;
        gazebo::physics::WorldPtr world;
        std::vector<gazebo::physics::ModelPtr> boxes;
        std::map<std::string, bool> toggled;
        std::map<std::string, bool> droneToggled;
        std::map<std::string, gazebo::transport::PublisherPtr> pubs;
        gazebo::physics::ModelPtr drone;
        bool droneBoxEnabled;
    };
} // namespace ariac_plugins

#endif // ARIAC_PLUGINS__AGV_PLUGIN_HPP_