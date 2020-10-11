/* Modifications and adaptations for use 
*		in the Lego Mindstorms EV3 simulation: 
 *	\author Steven Palma Morera
 *	\date 22th of Nov 2017
 */

#ifndef _WELCOME_PLUGIN_HH_
#define _WELCOME_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class WelcomePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: WelcomePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    	std::cerr<<"\n**********************************************************************\n**********************************************************************\n----------------------------------------------------------------------\n******************** Lego Mindstorms Education EV3 ******************* \n************************ Gazebo-ROS Simulation ***********************\n----------------------------------------------------------------------\n**********************************************************************\n**********************************************************************\n\n**********************************************************************   \n**********************************************************************\n----------------------------------------------------------------------\n********** Hello! Welcome to the lmev project, an effort to **********\n********** simulate a Lego Mindstorms Education EV3 Core Set *********\n********** 45544 in a compatible Gazebo and ROS environment. *********\n********** Futher information about installation, tutorials **********\n********** and basic use examples can be found in the user guide. ****\n----------------------------------------------------------------------\n**********************************************************************\n**********************************************************************\n\n**********************************************************************\n**********************************************************************\n----------------------------------------------------------------------\n********** Date created: 22 December, 2017 ***************************\n********** Date of last modification: 22 December, 2017 **************\n********** Mainteiner: Steven Palma Morera ***************************\n********** Contact email: imstevenpm.work@gmail.com ******************\n----------------------------------------------------------------------\n**********************************************************************\n**********************************************************************\n\n**********************************************************************\n----------------------------------------------------------------------\n************************* SIMULATION READY ***************************\n----------------------------------------------------------------------\n**********************************************************************\n";  
        
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(WelcomePlugin)
}
#endif
