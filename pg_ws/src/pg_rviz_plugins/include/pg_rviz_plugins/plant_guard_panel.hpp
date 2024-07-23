//
// Created by matej on 23/09/23.
//

#ifndef PLANT_GUARD_PLANT_GUARD_PANEL_HPP
#define PLANT_GUARD_PLANT_GUARD_PANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pg_msgs/srv/get_job_list.hpp"
#include "pg_msgs/srv/edit_job.hpp"
#include "pg_msgs/msg/bool_stamped.hpp"
#include "pg_msgs/msg/float64_stamped.hpp"

#include "pg_rviz_plugins/pump_crane_joystick.hpp"

#include <QSlider>
#include <QTableWidget>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>

namespace pg_rviz_plugins {

class PlantGuardPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  PlantGuardPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  void onInitialize() override;
  virtual void load( const rviz_common::Config& config ) override;
  virtual void save( rviz_common::Config config ) const override;
  virtual void setVisible(bool visible) override;

protected Q_SLOTS:
  void onPumpCraneJoystick(int direction, bool pump);
  void onPumpCraneJobSelected(int row, int column);
  void onPumpCraneJobEdited(int row, int column);
  void onPumpCraneJobDelete(int row, int column);
  void onCreateJobButtonClicked();
  void onCaptureButtonClicked();
  void refreshJobs();

protected:

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr sync_response_executor_;
  rclcpp::CallbackGroup::SharedPtr sync_response_cb_group_;

  rclcpp::Subscription<pg_msgs::msg::BoolStamped>::SharedPtr pc_pump_sub_;
  rclcpp::Subscription<pg_msgs::msg::Float64Stamped>::SharedPtr pc_angle_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pc_movement_dir_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pc_pump_pub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  rclcpp::Client<pg_msgs::srv::GetJobList>::SharedPtr get_jobs_cl_;
  rclcpp::Client<pg_msgs::srv::EditJob>::SharedPtr edit_job_cl_;

  PumpCraneJoystick* joystick_;
  QTableWidget* pc_jobs_table_;
  QPushButton* create_job_button_;
  QPushButton *capture_button_;
  QLabel* status_label_;

  std::mutex stored_msgs_mutex_;
  std_msgs::msg::Int32 dir_msg_;
  std_msgs::msg::Bool pump_msg_;

  QString current_cell_text_;
  bool editing_cells_internally_ = false;
  double last_pump_duration_ = std::numeric_limits<double>::infinity();
  double last_angle_ = std::numeric_limits<double>::infinity();
  rclcpp::Time last_pump_time_ = rclcpp::Time(0);

  template<class MsgT>
  typename MsgT::Response::SharedPtr callService(typename rclcpp::Client<MsgT>::SharedPtr client, const typename MsgT::Request::SharedPtr& request);

  void updateButtonsAvailability();
  bool isJobCellValid(int row, int column, std::string *error_msg = nullptr);

  class EditingCellGuard {
  public:
    EditingCellGuard(PlantGuardPanel *panel) : panel_(panel) {
      panel_->editing_cells_internally_ = true;
    }

    ~EditingCellGuard() {
      panel_->editing_cells_internally_ = false;
    }
  private:
    PlantGuardPanel *panel_;
  };

};

} // pg_rviz_plugins

#endif //PLANT_GUARD_PLANT_GUARD_PANEL_HPP
