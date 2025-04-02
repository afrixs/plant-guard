//
// Created by matej on 24/12/11.
//

#include "pg_rviz_plugins/moisture_sensor_panel.hpp"
#include "rviz_common/display_context.hpp"
#include "bagtube_rviz_plugins/bagtube_panel.hpp"
#include "pg_moisture_sensor_msgs/msg/config.hpp"
#include "pg_moisture_sensor_msgs/msg/job.hpp"
#include "rclcpp/serialization.hpp"
#include "pg_rviz_plugins/common.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMouseEvent>

#include <QProxyStyle>

#include <regex>

using namespace std::chrono_literals;

const auto LOGGER = rclcpp::get_logger("MoistureSensorPanel");

namespace pg_rviz_plugins {

const int MOISTURE_SENSORS_COUNT_PER_DEVICE = 8;

typedef enum class MSJobColumn {
  DEVICE = 0,
  NAME,
  MEASUREMENT_DURATION,
  TERMINATION_CONDITION,
  SCHEDULE,
  COUNT
};

QString toString(MSJobColumn column) {
  switch (column) {
    case MSJobColumn::DEVICE:
      return "Device";
    case MSJobColumn::NAME:
      return "Name";
    case MSJobColumn::MEASUREMENT_DURATION:
      return "Duration";
    case MSJobColumn::TERMINATION_CONDITION:
      return "Termination condition";
    case MSJobColumn::SCHEDULE:
      return "Schedule";
    default:
      return "UNKNOWN";
  }
}

typedef enum class MSDeviceColumn {
  MEASURE_NAME = 0,
  ZERO_MOISTURE_VALUE,
  FULL_MOISTURE_VALUE,
  MOISTURE,
  RAW_VALUE,
  COUNT
};

QString toString(MSDeviceColumn column) {
  switch (column) {
    case MSDeviceColumn::MEASURE_NAME:
      return "Measure name";
    case MSDeviceColumn::ZERO_MOISTURE_VALUE:
      return "Zero moisture";
    case MSDeviceColumn::FULL_MOISTURE_VALUE:
      return "Full moisture";
    case MSDeviceColumn::MOISTURE:
      return "Moisture";
    case MSDeviceColumn::RAW_VALUE:
      return "Raw value";
    default:
      return "UNKNOWN";
  }
}

MoistureSensor::MoistureSensor(const std::string &name, rclcpp::Node &node, MoistureSensorPanel *listener) {
  connect(this, SIGNAL(measurementReceived()), listener, SLOT(onMeasurementReceived()));
  measurement_sub_ = node.create_subscription<pg_msgs::msg::Float64ArrayStamped>(
      name + "/moistures_stream",
      rclcpp::QoS(1).best_effort(),
      [this](const pg_msgs::msg::Float64ArrayStamped::SharedPtr msg) {
        last_measurement_ = msg;
        while (!config_queue_.empty() && rclcpp::Time(msg->header.stamp) >= rclcpp::Time(config_queue_.front()->header.stamp)) {
          active_config_ = config_queue_.front();
          config_queue_.pop_front();
        }
        if (active_config_ && last_measurement_->header.config_id == active_config_->header.config_id &&
            rclcpp::Time(last_measurement_->header.stamp) >= rclcpp::Time(active_config_->header.stamp))
          reconstructSensorValues();
      }
  );
  config_sub_ = node.create_subscription<pg_moisture_sensor_msgs::msg::ConfigStamped>(
      name + "/config_stamped_stream",
      rclcpp::QoS(1).transient_local(),
      [this](const pg_moisture_sensor_msgs::msg::ConfigStamped::SharedPtr msg) {
        if (last_measurement_ &&
            rclcpp::Time(last_measurement_->header.stamp) >= rclcpp::Time(msg->header.stamp)) {
          active_config_ = msg;
          config_queue_.clear();
          if (last_measurement_->header.config_id == active_config_->header.config_id)
            reconstructSensorValues();
        } else
          config_queue_.push_back(msg);
      }
  );
}

void MoistureSensor::reconstructSensorValues() {
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    last_moistures_raw_vals_.clear();
    last_moistures_raw_vals_.reserve(active_config_->config.sensors.size());
    size_t msg_data_i = 0;
    for (size_t i = 0; i < active_config_->config.sensors.size(); i++) {
      auto &sensor = active_config_->config.sensors[i];
      if (sensor.measure_name.empty())
        last_moistures_raw_vals_.emplace_back(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
      else {
        double moisture = msg_data_i < last_measurement_->data.size() ? last_measurement_->data[msg_data_i++] : std::numeric_limits<double>::quiet_NaN();
        if (sensor.zero_moisture_value == sensor.full_moisture_value)
          last_moistures_raw_vals_.emplace_back(std::numeric_limits<double>::quiet_NaN(), moisture);
        else {
          double raw_val = moisture*(sensor.full_moisture_value - sensor.zero_moisture_value) + sensor.zero_moisture_value;
          last_moistures_raw_vals_.emplace_back(moisture, raw_val);
        }
      }
    }
  }

  emit measurementReceived();
}

bool MoistureSensor::hasData() const {
  std::unique_lock<std::mutex> lock(data_mutex_);
  return !last_moistures_raw_vals_.empty();
}

std::vector<std::pair<double, double>> MoistureSensor::getMoisturesRawVals() const {
  std::unique_lock<std::mutex> lock(data_mutex_);
  return last_moistures_raw_vals_;
}

MoistureSensorPanel::MoistureSensorPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  auto layout = new QVBoxLayout;

  auto tables_layout = new QHBoxLayout;

  jobs_table_ = new bagtube_rviz_plugins::BagtubeTableWidget;
  jobs_table_->setColumnCount((int)MSJobColumn::COUNT);
  QStringList headers;
  for (int i = 0; i < (int)MSJobColumn::COUNT; i++)
    headers << toString((MSJobColumn)i);
  jobs_table_->setHorizontalHeaderLabels(headers);
  jobs_table_->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  connect(jobs_table_, SIGNAL(cellClicked(int,int)), this, SLOT(onJobSelected(int,int)));
  connect(jobs_table_, SIGNAL(deletePressed(int,int)), this, SLOT(onJobDelete(int,int)));
  connect(jobs_table_, SIGNAL(cellChanged(int,int)), this, SLOT(onJobEdited(int,int)));
  tables_layout->addWidget(jobs_table_);

  auto device_table_layout = new QVBoxLayout;
  auto device_picker_layout = new QHBoxLayout;

  device_picker_ = new QComboBox;
  device_picker_->addItem("--New--");
  connect(device_picker_, SIGNAL(currentIndexChanged(int)), this, SLOT(onDeviceSelected(int)));
  device_picker_layout->addWidget(device_picker_);

  device_name_edit_ = new QLineEdit;
  device_name_edit_->setPlaceholderText("Device name");
  connect(device_name_edit_, SIGNAL(textChanged(QString)), this, SLOT(onDeviceNameEdited(QString)));
  device_picker_layout->addWidget(device_name_edit_);

  device_name_confirm_button_ = new QPushButton("Create");
  connect(device_name_confirm_button_, SIGNAL(clicked()), this, SLOT(onDeviceNameConfirmButtonClicked()));
  device_picker_layout->addWidget(device_name_confirm_button_);

  device_table_layout->addLayout(device_picker_layout);

  devices_table_ = new bagtube_rviz_plugins::BagtubeTableWidget;
  devices_table_->setColumnCount((int)MSDeviceColumn::COUNT);
  headers.clear();
  for (int i = 0; i < (int)MSDeviceColumn::COUNT; i++)
    headers << toString((MSDeviceColumn)i);
  devices_table_->setHorizontalHeaderLabels(headers);
  devices_table_->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  connect(devices_table_, SIGNAL(cellClicked(int,int)), this, SLOT(onDeviceConfigSelected(int,int)));
  connect(devices_table_, SIGNAL(cellChanged(int,int)), this, SLOT(onDeviceConfigEdited(int,int)));
  device_table_layout->addWidget(devices_table_);

  tables_layout->addLayout(device_table_layout);
  layout->addLayout(tables_layout);

  auto button_layout = new QHBoxLayout;
  create_job_button_ = new QPushButton("Create job");
  create_job_button_->setEnabled(false);
  connect(create_job_button_, SIGNAL(clicked()), this, SLOT(onCreateButtonClicked()));
  button_layout->addWidget(create_job_button_);

  capture_button_ = new QPushButton("Capture");
  capture_button_->setEnabled(false);
  connect(capture_button_, SIGNAL(clicked()), this, SLOT(onCaptureButtonClicked()));
  button_layout->addWidget(capture_button_);

  auto refresh_button = new QPushButton("Refresh");
  connect(refresh_button, SIGNAL(clicked()), this, SLOT(refresh()));
  button_layout->addWidget(refresh_button);

  layout->addLayout(button_layout);

  status_label_ = new QLabel;
  layout->addWidget(status_label_);

  setLayout(layout);

  new_device_config_.sensors.resize(MOISTURE_SENSORS_COUNT_PER_DEVICE);
  onDeviceSelected(0);
}

void MoistureSensorPanel::onInitialize() {
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  auto node_lock = rviz_ros_node_.lock();
  auto node = node_lock->get_raw_node();

  sync_response_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sync_response_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  sync_response_executor_->add_callback_group(sync_response_cb_group_, node->get_node_base_interface());

  get_jobs_cl_ = node->create_client<pg_msgs::srv::GetJobList>(
      "get_job_list",
      rclcpp::ServicesQoS(),
      sync_response_cb_group_
  );
  edit_job_cl_ = node->create_client<pg_msgs::srv::EditJob>(
      "edit_job",
      rclcpp::ServicesQoS(),
      sync_response_cb_group_
  );
  get_devices_cl_ = node->create_client<pg_msgs::srv::GetDeviceList>(
      "get_device_list",
      rclcpp::ServicesQoS(),
      sync_response_cb_group_
  );
  edit_device_cl_ = node->create_client<pg_msgs::srv::EditDevice>(
      "edit_device",
      rclcpp::ServicesQoS(),
      sync_response_cb_group_
  );

  refresh();
}

void MoistureSensorPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void MoistureSensorPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
}

void MoistureSensorPanel::refresh() {
  refreshDevices();
  refreshJobs();
}

void MoistureSensorPanel::refreshDevices() {
  auto request = std::make_shared<pg_msgs::srv::GetDeviceList::Request>();
  request->type = pg_moisture_sensor_msgs::msg::Config::DEVICE_TYPE;
  auto response = callService<pg_msgs::srv::GetDeviceList>(get_devices_cl_, request, *sync_response_executor_);
  if (!response) {
    RCLCPP_ERROR(LOGGER, "Failed to get device list");
    status_label_->setText("Failed to get device list");
    return;
  }

  EditingCellGuard guard(this);
  QString prev_selected_device = device_picker_->currentText();

  device_picker_->clear();
  device_picker_->addItem("--New--");
  std::set<std::string> devices_to_remove;
  for (auto &device : devices_)
    devices_to_remove.insert(device.first);
  bool has_prev_selected_device = prev_selected_device == device_picker_->itemText(0);
  for (auto &device_msg : response->devices) {
    device_picker_->addItem(QString::fromStdString(device_msg.name));
    auto device = devices_[device_msg.name];
    if (!device) {
      device = std::make_shared<MoistureSensor>(device_msg.name, *rviz_ros_node_.lock()->get_raw_node(), this);
      devices_[device_msg.name] = device;
    }
    else
      devices_to_remove.erase(device_msg.name);
    deserialize_message(device_msg.config_msg_data, device->config);
    if (device_msg.name == prev_selected_device.toStdString())
      has_prev_selected_device = true;
  }

  for (auto &device_name : devices_to_remove)
    devices_.erase(device_name);

  if (has_prev_selected_device)
    device_picker_->setCurrentText(prev_selected_device);
  else {
    // device_name_edit_->clear();
    // device_picker_->setCurrentIndex(0);
    device_picker_->setCurrentIndex(device_picker_->count() - 1);
    device_name_edit_->text() = device_picker_->currentText();
  }

  onDeviceSelected(device_picker_->currentIndex());
  status_label_->setText("Devices refreshed");
}

void MoistureSensorPanel::onDeviceSelected(int index) {
  if (editing_cells_internally_)
    return;
  pg_moisture_sensor_msgs::msg::Config *config;
  std::shared_ptr<MoistureSensor> device;
  if (index == 0) {
    if (!editing_cells_internally_) {
      EditingCellGuard guard(this);
      device_name_edit_->clear();
    }
    config = &new_device_config_;
  }
  else {
    EditingCellGuard guard(this);
    auto device_name = device_picker_->currentText().toStdString();
    device = devices_[device_name];
    device_name_edit_->setText(QString::fromStdString(device_name));
    config = &device->config;
  }

  EditingCellGuard guard(this);
  devices_table_->setRowCount((int)config->sensors.size());
  std::vector<std::pair<double, double>> moistures_raw_vals;
  for (int i = 0; i < devices_table_->rowCount(); i++) {
    const auto &sensor = config->sensors[i];
    devices_table_->setItem(i, (int)MSDeviceColumn::MEASURE_NAME, new QTableWidgetItem(QString::fromStdString(sensor.measure_name)));
    devices_table_->setItem(i, (int)MSDeviceColumn::ZERO_MOISTURE_VALUE, new QTableWidgetItem(QString::number(sensor.zero_moisture_value)));
    devices_table_->setItem(i, (int)MSDeviceColumn::FULL_MOISTURE_VALUE, new QTableWidgetItem(QString::number(sensor.full_moisture_value)));
    if (!device) {
      auto moisture_item = new QTableWidgetItem();
      moisture_item->setFlags(moisture_item->flags() & ~Qt::ItemIsEditable);
      devices_table_->setItem(i, (int)MSDeviceColumn::MOISTURE, moisture_item);
      auto raw_value_item = new QTableWidgetItem();
      raw_value_item->setFlags(raw_value_item->flags() & ~Qt::ItemIsEditable);
      devices_table_->setItem(i, (int)MSDeviceColumn::RAW_VALUE, raw_value_item);
    }
  }
  if (device)
    device->measurementReceived();
  updateButtonsAvailability();
}

void MoistureSensorPanel::onDeviceConfigSelected(int row, int column) {
  if (row != -1)
    current_cell_text_ = devices_table_->item(row, column)->text();
  else
    devices_table_->setCurrentCell(-1, -1);
  updateButtonsAvailability();
}

bool MoistureSensorPanel::isDeviceCellValid(int row, int column, std::string *error_msg) {
  std::string text = devices_table_->item(row, column)->text().toStdString();
  switch ((MSDeviceColumn)column) {
    case MSDeviceColumn::MEASURE_NAME:
      // TODO: validate measure name
      return true;
    case MSDeviceColumn::ZERO_MOISTURE_VALUE:
    case MSDeviceColumn::FULL_MOISTURE_VALUE: {
      if (double value; parse_double(text, value))
        return true;
      else {
        if (error_msg)
          *error_msg = "Invalid number";
        return false;
      }
    }
    default:
      return true;
  }
}

void MoistureSensorPanel::onDeviceConfigEdited(int row, int column) {
  if (editing_cells_internally_)
    return;
  auto &config = device_picker_->currentIndex() == 0 ? new_device_config_ : devices_[device_picker_->currentText().toStdString()]->config;
  if (!isDeviceCellValid(row, column)) {
    status_label_->setText("Invalid value in " + toString((MSDeviceColumn)column) + " column");
    devices_table_->setItem(row, column, new QTableWidgetItem(current_cell_text_));
    return;
  }

  auto prev_config = config;
  EditingCellGuard guard(this);
  switch ((MSDeviceColumn)column) {
    case MSDeviceColumn::MEASURE_NAME:
      config.sensors[row].measure_name = devices_table_->item(row, column)->text().toStdString();
      break;
    case MSDeviceColumn::ZERO_MOISTURE_VALUE:
      config.sensors[row].zero_moisture_value = devices_table_->item(row, column)->text().toDouble();
      break;
    case MSDeviceColumn::FULL_MOISTURE_VALUE:
      config.sensors[row].full_moisture_value = devices_table_->item(row, column)->text().toDouble();
      break;
    default:
      return;
  }

  if (device_picker_->currentIndex() == 0) {
    updateButtonsAvailability();
    return;
  }

  auto request = std::make_shared<pg_msgs::srv::EditDevice::Request>();
  request->operation = pg_msgs::srv::EditDevice::Request::MODIFY;
  request->device.name = device_picker_->currentText().toStdString();
  request->device.type = pg_moisture_sensor_msgs::msg::Config::DEVICE_TYPE;
  serialize_message(config, request->device.config_msg_data);

  auto response = callService<pg_msgs::srv::EditDevice>(edit_device_cl_, request, *sync_response_executor_);
  if (!response || !response->success) {
    RCLCPP_ERROR(LOGGER, "Failed to edit device %s: %s", request->device.name.c_str(), response ? response->message.c_str() : "no response");
    status_label_->setText("Failed to edit device " + QString::fromStdString(request->device.name) + ": " + QString::fromStdString(response ? response->message : "no response"));
    config = prev_config;
    devices_table_->setItem(row, column, new QTableWidgetItem(current_cell_text_));
    return;
  }
  status_label_->setText("Device " + QString::fromStdString(request->device.name) + " edited");
  updateButtonsAvailability();
}

void MoistureSensorPanel::onDeviceNameEdited(const QString &) {
  updateButtonsAvailability();
}

void MoistureSensorPanel::onMeasurementReceived() {
  if (device_picker_->currentIndex() == 0)
    return;
  auto device = devices_[device_picker_->currentText().toStdString()];
  if ((MoistureSensor *)sender() != device.get())
    return;
  auto mrvs = device->getMoisturesRawVals();
  if (devices_table_->rowCount() != (int)mrvs.size())
    return;
  EditingCellGuard guard(this);
  for (int i = 0; i < devices_table_->rowCount(); i++) {
    auto mr = mrvs[i];
    devices_table_->item(i, (int)MSDeviceColumn::MOISTURE)->setText(std::isnan(mr.first) ? "" : QString::number(mr.first*100.0, 'g', 3)+'%');
    devices_table_->item(i, (int)MSDeviceColumn::RAW_VALUE)->setText(std::isnan(mr.second) ? "" : QString::number(mr.second));
  }
}

void MoistureSensorPanel::refreshJobs() {
  auto request = std::make_shared<pg_msgs::srv::GetJobList::Request>();
  auto response = callService<pg_msgs::srv::GetJobList>(get_jobs_cl_, request, *sync_response_executor_);
  if (!response) {
    RCLCPP_ERROR(LOGGER, "Failed to get job list");
    status_label_->setText("Failed to get job list");
    return;
  }

  EditingCellGuard guard(this);
  int prev_selected_row = jobs_table_->currentRow();

  bool had_incomplete_job = false;
  if (jobs_table_->rowCount() > 0)
    for (int j = 0; j < jobs_table_->columnCount(); j++)
      if (jobs_table_->item(jobs_table_->rowCount() - 1, j)->text().isEmpty()) {
        had_incomplete_job = true;
        break;
      }
  if (had_incomplete_job)
    for (int i = jobs_table_->rowCount() - 2; i >= 0; i--)
      jobs_table_->removeRow(i);
  else
    jobs_table_->setRowCount(0);

  for (int i = 0; i < response->jobs.size(); i++) {
    auto &job = response->jobs[i];
    if (job.device.name != "moisture_sensor" || job.device.type != pg_moisture_sensor_msgs::msg::Config::DEVICE_TYPE)
      continue;

    pg_moisture_sensor_msgs::msg::Job ms_job;
    deserialize_message(job.config_msg_data, ms_job);

    jobs_table_->insertRow(i);
    jobs_table_->setItem(i, (int)MSJobColumn::DEVICE, new QTableWidgetItem(QString::fromStdString(job.device.name)));
    jobs_table_->setItem(i, (int)MSJobColumn::NAME, new QTableWidgetItem(QString::fromStdString(job.name)));
    jobs_table_->setItem(i, (int)MSJobColumn::MEASUREMENT_DURATION, new QTableWidgetItem(QString::number(ms_job.measurement_duration)));
    jobs_table_->setItem(i, (int)MSJobColumn::TERMINATION_CONDITION, new QTableWidgetItem(QString::fromStdString(ms_job.termination_condition)));
    jobs_table_->setItem(i, (int)MSJobColumn::SCHEDULE, new QTableWidgetItem(QString::fromStdString(job.crontab_schedule)));
  }

  if (prev_selected_row >= 0)
    jobs_table_->selectRow(std::min(jobs_table_->rowCount() - 1, prev_selected_row));
}

void MoistureSensorPanel::onJobSelected(int row, int column) {
  if (row != -1)
    current_cell_text_ = jobs_table_->item(row, column)->text();
  else
    jobs_table_->setCurrentCell(-1, -1);
  updateButtonsAvailability();
}

bool MoistureSensorPanel::isJobCellValid(int row, int column, std::string *error_msg) {
  std::string text = jobs_table_->item(row, column)->text().toStdString();
  if (text.empty())
    return false;
  switch ((MSJobColumn)column) {
    case MSJobColumn::DEVICE:
      if (text != "moisture_sensor") {  // for now, only moisture_sensor is supported
        if (error_msg)
          *error_msg = "Invalid device (only moisture_sensor is supported for now)";
        return false;
      }
      return true;
    case MSJobColumn::NAME: {
      if (!std::regex_match(text, std::regex(ALLOWED_NAMING_PATTERN, std::regex_constants::ECMAScript))) {
        if (error_msg)
          *error_msg = "Invalid name";
        return false;
      }
      bool unique = true;
      for (int i = 0; i < jobs_table_->rowCount(); i++) {
        if (i == row)
          continue;
        if (jobs_table_->item(i, (int)MSJobColumn::NAME)->text().toStdString() == text) {
          unique = false;
          if (error_msg)
            *error_msg = "Name must be unique";
          break;
        }
      }
      return unique;
    }
    case MSJobColumn::MEASUREMENT_DURATION: {
      if (double value; parse_double(text, value) && value >= 0)
        return true;
      else {
        if (error_msg)
          *error_msg = "Invalid number";
        return false;
      }
    }
    case MSJobColumn::TERMINATION_CONDITION: {
      // TODO?
      return true;
    }
    case MSJobColumn::SCHEDULE: {
      if (!std::regex_match(text, std::regex(CRONTAB_SCHEDULE_PATTERN, std::regex_constants::ECMAScript))) {
        if (error_msg)
          *error_msg = "Invalid crontab schedule";
        return false;
      }
      return true;
    }
    default:
      return true;
  }
}

void MoistureSensorPanel::onJobEdited(int row, int column) {
  if (editing_cells_internally_)
    return;
  EditingCellGuard guard(this);
  std::string error_msg;
  if (!isJobCellValid(row, column, &error_msg)) {
    status_label_->setText("Invalid value in " + toString((MSJobColumn)column) + " column: " + QString::fromStdString(error_msg));
    jobs_table_->setItem(row, column, new QTableWidgetItem(current_cell_text_));
    return;
  }

  bool is_complete_row = true;
  for (int i = 0; i < jobs_table_->columnCount(); i++)
    if (!jobs_table_->item(row, i) || jobs_table_->item(row, i)->text().isEmpty()) {
      is_complete_row = false;
      break;
    }
  if (!is_complete_row)
    return;

  auto request = std::make_shared<pg_msgs::srv::EditJob::Request>();
  request->operation = pg_msgs::srv::EditJob::Request::ADD_OR_MODIFY;
  request->job.device.name = jobs_table_->item(row, (int)MSJobColumn::DEVICE)->text().toStdString();
  request->job.device.type = pg_moisture_sensor_msgs::msg::Config::DEVICE_TYPE;
  request->job.name = jobs_table_->item(row, (int)MSJobColumn::NAME)->text().toStdString();
  pg_moisture_sensor_msgs::msg::Job ms_job;
  ms_job.measurement_duration = std::stod(jobs_table_->item(row, (int)MSJobColumn::MEASUREMENT_DURATION)->text().toStdString());
  ms_job.termination_condition = std::stod(jobs_table_->item(row, (int)MSJobColumn::TERMINATION_CONDITION)->text().toStdString());
  serialize_message(ms_job, request->job.config_msg_data);
  request->job.crontab_schedule = jobs_table_->item(row, (int)MSJobColumn::SCHEDULE)->text().toStdString();

  auto response = callService<pg_msgs::srv::EditJob>(edit_job_cl_, request, *sync_response_executor_);
  if (!response || !response->success) {
    RCLCPP_ERROR(LOGGER, "Failed to edit job %s: %s", request->job.name.c_str(), response ? response->message.c_str() : "no response");
    status_label_->setText("Failed to edit job " + QString::fromStdString(request->job.name) + ": " + QString::fromStdString(response ? response->message : "no response"));
    jobs_table_->setItem(row, column, new QTableWidgetItem(current_cell_text_));
    return;
  }
  status_label_->setText("Job " + QString::fromStdString(request->job.name) + " edited");
  updateButtonsAvailability();
}

void MoistureSensorPanel::onJobDelete(int row, int /*column*/) {
  bool deleting_complete_row = true;
  for (int i = 0; i < jobs_table_->columnCount(); i++)
    if (jobs_table_->item(row, i)->text().isEmpty()) {
      deleting_complete_row = false;
      break;
    }
  if (deleting_complete_row) {
    RCLCPP_INFO(LOGGER, "Deleting job %s", jobs_table_->item(row, (int)MSJobColumn::NAME)->text().toStdString().c_str());
    auto request = std::make_shared<pg_msgs::srv::EditJob::Request>();
    request->operation = pg_msgs::srv::EditJob::Request::DELETE;
    request->job.device.name = jobs_table_->item(row, (int)MSJobColumn::DEVICE)->text().toStdString();
    request->job.name = jobs_table_->item(row, (int)MSJobColumn::NAME)->text().toStdString();

    auto response = callService<pg_msgs::srv::EditJob>(edit_job_cl_, request, *sync_response_executor_);
    if (!response) {
      RCLCPP_ERROR(LOGGER, "Failed to delete job %s", request->job.name.c_str());
      status_label_->setText("Failed to delete job " + QString::fromStdString(request->job.name));
      return;
    }
  }
  jobs_table_->removeRow(row);
  updateButtonsAvailability();
}

void MoistureSensorPanel::updateButtonsAvailability()
{
  bool can_capture = false;
  device_name_confirm_button_->setText(device_picker_->currentIndex() == 0 ? "Create" : "Rename");
  if (device_name_edit_->text().length() == 0 ||
      !std::regex_match(device_name_edit_->text().toStdString(), std::regex(ALLOWED_NAMING_PATTERN, std::regex_constants::ECMAScript))) {
    device_name_confirm_button_->setEnabled(false);
  }
  else {
    bool unique = true;
    for (int i = 0; i < device_picker_->count(); i++) {
      if (device_picker_->itemText(i) == device_name_edit_->text()) {
        unique = false;
        break;
      }
    }
    bool valid_data = true;
    for (int i = 0; i < devices_table_->rowCount(); i++) {
      if (devices_table_->item(i, (int)MSDeviceColumn::MEASURE_NAME)->text().isEmpty())
        continue;

      if (double value;
          !parse_double(devices_table_->item(i, (int)MSDeviceColumn::ZERO_MOISTURE_VALUE)->text().toStdString(), value) ||
          !parse_double(devices_table_->item(i, (int)MSDeviceColumn::FULL_MOISTURE_VALUE)->text().toStdString(), value)) {
        valid_data = false;
        break;
      }
    }
    device_name_confirm_button_->setEnabled(unique && valid_data);
  }
  if (devices_table_->hasFocus()) {
    auto column = (MSDeviceColumn)devices_table_->currentColumn();
    if ((column == MSDeviceColumn::ZERO_MOISTURE_VALUE || column == MSDeviceColumn::FULL_MOISTURE_VALUE) && device_picker_->currentIndex() != 0) {
      auto mrvs = devices_[device_picker_->currentText().toStdString()]->getMoisturesRawVals();
      if (mrvs.size() > (size_t)devices_table_->currentRow() && !std::isnan(mrvs[devices_table_->currentRow()].second))
        can_capture = true;
    }
  }

  // create_job_button_->setText("Create job");
  // bool can_create = true; // only Create device is supported currently
  // for (int i = 0; i < jobs_table_->rowCount(); i++) {
  //   for (int j = 0; j < jobs_table_->columnCount(); j++)
  //     if (jobs_table_->item(i, j)->text().isEmpty()) {
  //       can_create = false;
  //       break;
  //     }
  //   if (!can_create)
  //     break;
  // }
  // create_job_button_->setEnabled(can_create);
  capture_button_->setEnabled(can_capture);
}

void MoistureSensorPanel::onDeviceNameConfirmButtonClicked() {
  if (device_picker_->currentIndex() == 0) {
    auto &config = new_device_config_;
    auto request = std::make_shared<pg_msgs::srv::EditDevice::Request>();
    request->operation = pg_msgs::srv::EditDevice::Request::ADD;
    request->device.name = device_name_edit_->text().toStdString();
    request->device.type = pg_moisture_sensor_msgs::msg::Config::DEVICE_TYPE;
    serialize_message(config, request->device.config_msg_data);

    auto response = callService<pg_msgs::srv::EditDevice>(edit_device_cl_, request, *sync_response_executor_);
    if (!response) {
      RCLCPP_ERROR(LOGGER, "Failed to create device %s", request->device.name.c_str());
      status_label_->setText("Failed to create device" + QString::fromStdString(request->device.name));
      return;
    }

    auto node_lock = rviz_ros_node_.lock();
    auto node = node_lock->get_raw_node();
    auto device = std::make_shared<MoistureSensor>(request->device.name, *node, this);
    device->config = config;
    devices_[device_name_edit_->text().toStdString()] = device;

    new_device_config_.sensors.clear();
    new_device_config_.sensors.resize(MOISTURE_SENSORS_COUNT_PER_DEVICE);

    device_picker_->addItem(device_name_edit_->text());
    device_picker_->setCurrentIndex(device_picker_->count() - 1);
  }
  else {
    auto request = std::make_shared<pg_msgs::srv::EditDevice::Request>();
    request->operation = pg_msgs::srv::EditDevice::Request::MODIFY;
    request->name_before_renaming = device_picker_->currentText().toStdString();
    request->device.name = device_name_edit_->text().toStdString();
    request->device.type = pg_moisture_sensor_msgs::msg::Config::DEVICE_TYPE;
    auto device = devices_[request->name_before_renaming];
    serialize_message(device->config, request->device.config_msg_data);

    auto response = callService<pg_msgs::srv::EditDevice>(edit_device_cl_, request, *sync_response_executor_);
    if (!response) {
      RCLCPP_ERROR(LOGGER, "Failed to rename device %s", request->device.name.c_str());
      status_label_->setText("Failed to create device" + QString::fromStdString(request->device.name));
      return;
    }

    devices_[request->device.name] = devices_[request->name_before_renaming];
    devices_.erase(request->name_before_renaming);

    device_picker_->setItemText(device_picker_->currentIndex(), device_name_edit_->text());
  }

  updateButtonsAvailability();
}

void MoistureSensorPanel::onCreateButtonClicked() {
  // EditingCellGuard guard(this);
  // jobs_table_->insertRow(jobs_table_->rowCount());
  // jobs_table_->setItem(jobs_table_->rowCount() - 1, (int)MSColumn::DEVICE, new QTableWidgetItem("moisture_sensor"));
  //
  // int job_number = jobs_table_->rowCount();
  // bool unique;
  // do {
  //   unique = true;
  //   for (int i = 0; i < jobs_table_->rowCount() - 1; i++) {
  //     if (jobs_table_->item(i, (int)MSColumn::NAME)->text().toStdString() == "job" + std::to_string(job_number)) {
  //       job_number++;
  //       unique = false;
  //       break;
  //     }
  //   }
  // } while (!unique);
  // jobs_table_->setItem(jobs_table_->rowCount() - 1, (int)MSColumn::NAME, new QTableWidgetItem("job" + QString::number(job_number)));
  //
  // for (int i = (int)MSColumn::NAME + 1; i < (int)MSColumn::COUNT; i++)
  //   jobs_table_->setItem(jobs_table_->rowCount() - 1, i, new QTableWidgetItem(""));

  // create_job_button_->setEnabled(false);
}

void MoistureSensorPanel::onCaptureButtonClicked() {
  auto column = (MSDeviceColumn)devices_table_->currentColumn();
  if (column != MSDeviceColumn::ZERO_MOISTURE_VALUE && column != MSDeviceColumn::FULL_MOISTURE_VALUE)
    return;
  auto device = devices_[device_picker_->currentText().toStdString()];
  auto mrvs = device->getMoisturesRawVals();
  if (devices_table_->currentRow() >= (int)mrvs.size())
    return;
  auto mr = mrvs[devices_table_->currentRow()];
  if (std::isnan(mr.second))
    return;
  devices_table_->item(devices_table_->currentRow(), (int)column)->setText(QString::number(mr.second));
}

} // pg_rviz_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(pg_rviz_plugins::MoistureSensorPanel, rviz_common::Panel)
