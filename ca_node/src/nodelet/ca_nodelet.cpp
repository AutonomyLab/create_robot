
/**
 * @file /ca_node/src/nodelet/ca_nodelet.cpp
 *
 * @brief Implementation for the ROS Create nodelet
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "create_driver/create_driver.h"


namespace create
{

class CreateNodelet : public nodelet::Nodelet
{
public:
  CreateNodelet() : shutdown_requested_(false) {};
  ~CreateNodelet()
  {
    NODELET_DEBUG_STREAM("Create : waiting for update thread to finish.");
    shutdown_requested_ = true;
    update_thread_.join();
  }
  virtual void onInit()
  {
    NODELET_DEBUG_STREAM("Create : initialising nodelet...");
    create_.reset(new CreateDriver(this->getNodeHandle(),this->getPrivateNodeHandle()));
    update_thread_.start(&CreateNodelet::update, *this);
  }
private:
  void update()
  {
	create_->spin();
  }

  boost::shared_ptr<CreateDriver> create_;
  ecl::Thread update_thread_;
  bool shutdown_requested_;
};

} // namespace ca

PLUGINLIB_EXPORT_CLASS(create::CreateNodelet, nodelet::Nodelet);
