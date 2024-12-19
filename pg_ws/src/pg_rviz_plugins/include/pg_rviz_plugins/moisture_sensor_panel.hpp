//
// Created by matej on 24/12/11.
//

#ifndef PLANT_GUARD_MOISTURE_SENSOR_PANEL_HPP
#define PLANT_GUARD_MOISTURE_SENSOR_PANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pg_msgs/srv/get_job_list.hpp"
#include "pg_msgs/srv/edit_job.hpp"
#include "pg_msgs/srv/get_device_list.hpp"
#include "pg_msgs/srv/edit_device.hpp"
#include "pg_msgs/msg/float64_array_stamped.hpp"
#include "pg_moisture_sensor_msgs/msg/config.hpp"
#include "pg_moisture_sensor_msgs/msg/config_stamped.hpp"

#include <QSlider>
#include <QTableWidget>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>

namespace pg_rviz_plugins {

class MoistureSensorPanel;

class MoistureSensor: QObject {
  Q_OBJECT
public:
  pg_moisture_sensor_msgs::msg::Config config;

  MoistureSensor() = default;
  MoistureSensor(const std::string &name, rclcpp::Node &node, MoistureSensorPanel *listener);

  bool hasData() const;
  std::vector<std::pair<double, double>> getMoisturesRawVals() const;

  Q_SIGNALS:
  void measurementReceived();

protected:
  std::deque<pg_moisture_sensor_msgs::msg::ConfigStamped::SharedPtr> config_queue_;
  pg_msgs::msg::Float64ArrayStamped::SharedPtr last_measurement_;
  pg_moisture_sensor_msgs::msg::ConfigStamped::SharedPtr active_config_;
  rclcpp::Subscription<pg_msgs::msg::Float64ArrayStamped>::SharedPtr measurement_sub_;
  rclcpp::Subscription<pg_moisture_sensor_msgs::msg::ConfigStamped>::SharedPtr config_sub_;
  std::vector<std::pair<double, double>> last_moistures_raw_vals_;

  mutable std::mutex data_mutex_;

  void reconstructSensorValues();
};

class MoistureSensorPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  MoistureSensorPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  void onInitialize() override;
  virtual void load( const rviz_common::Config& config ) override;
  virtual void save( rviz_common::Config config ) const override;

protected Q_SLOTS:
  void onJobSelected(int row, int column);
  void onJobEdited(int row, int column);
  void onJobDelete(int row, int column);
  void onDeviceConfigSelected(int row, int column);
  void onDeviceConfigEdited(int row, int column);
  void onCreateButtonClicked();
  void onCaptureButtonClicked();
  void refresh();
  void onDeviceNameConfirmButtonClicked();
  void onDeviceSelected(int index);
  void onDeviceNameEdited(const QString &text);
  void onMeasurementReceived();
protected:

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr sync_response_executor_;
  rclcpp::CallbackGroup::SharedPtr sync_response_cb_group_;

  rclcpp::Client<pg_msgs::srv::GetJobList>::SharedPtr get_jobs_cl_;
  rclcpp::Client<pg_msgs::srv::EditJob>::SharedPtr edit_job_cl_;
  rclcpp::Client<pg_msgs::srv::GetDeviceList>::SharedPtr get_devices_cl_;
  rclcpp::Client<pg_msgs::srv::EditDevice>::SharedPtr edit_device_cl_;

  QTableWidget* jobs_table_;
  QPushButton* create_job_button_;
  QPushButton *capture_button_;
  QTableWidget* devices_table_;
  QComboBox* device_picker_;
  QLineEdit* device_name_edit_;
  QPushButton* device_name_confirm_button_;
  QLabel* status_label_;

  QString current_cell_text_;
  int editing_cells_internally_ = 0;

  std::map<std::string, std::shared_ptr<MoistureSensor>> devices_;
  pg_moisture_sensor_msgs::msg::Config new_device_config_;

  void updateButtonsAvailability();
  bool isDeviceCellValid(int row, int column, std::string *error_msg = nullptr);
  void refreshDevices();
  bool isJobCellValid(int row, int column, std::string *error_msg = nullptr);
  void refreshJobs();

  class EditingCellGuard {
  public:
    EditingCellGuard(MoistureSensorPanel *panel) : panel_(panel) {
      panel_->editing_cells_internally_++;
    }

    ~EditingCellGuard() {
      panel_->editing_cells_internally_--;
    }
  private:
    MoistureSensorPanel *panel_;
  };

};

} // pg_rviz_plugins

#endif //PLANT_GUARD_MOISTURE_SENSOR_PANEL_HPP
