//
// Created by matej on 23/09/23.
//

#include "pg_rviz_plugins/plant_guard_panel.hpp"
#include "rviz_common/display_context.hpp"
#include "bagtube_rviz_plugins/bagtube_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMouseEvent>

#include <QProxyStyle>

#include <regex>

using namespace std::chrono_literals;

const auto LOGGER = rclcpp::get_logger("PlantGuardPanel");
const std::string ALLOWED_NAMING_PATTERN = "(?!^__)[a-zA-Z0-9_ -]*";
const std::string CRONTAB_SCHEDULE_PATTERN =  // "(\\*|(?:\\*|(?:[0-9]|(?:[1-5][0-9])))\\/(?:[0-9]|(?:[1-5][0-9]))|"  // second
                                              // "(?:[0-9]|(?:[1-5][0-9]))(?:(?:\\-[0-9]|\\-(?:[1-5][0-9]))?|(?:\\,(?:[0-9]|(?:[1-5][0-9])))*)) "
                                             "(\\*|(?:\\*|(?:[0-9]|(?:[1-5][0-9])))\\/(?:[0-9]|(?:[1-5][0-9]))|(?:[0-9]|"  // minute
                                             "(?:[1-5][0-9]))(?:(?:\\-[0-9]|\\-(?:[1-5][0-9]))?|(?:\\,(?:[0-9]|(?:[1-5][0-9])))*)) "
                                             "(\\*|(?:\\*|(?:\\*|(?:[0-9]|1[0-9]|2[0-3])))\\/(?:[0-9]|1[0-9]|2[0-3])|"  // hour
                                             "(?:[0-9]|1[0-9]|2[0-3])(?:(?:\\-(?:[0-9]|1[0-9]|2[0-3]))?|(?:\\,(?:[0-9]|1[0-9]|2[0-3]))*)) "
                                             "(\\*|\\?|L(?:W|\\-(?:[1-9]|(?:[12][0-9])|3[01]))?|(?:[1-9]|(?:[12][0-9])|"  // day_of_month
                                             "3[01])(?:W|\\/(?:[1-9]|(?:[12][0-9])|3[01]))?|(?:[1-9]|(?:[12][0-9])|3[01])"
                                             "(?:(?:\\-(?:[1-9]|(?:[12][0-9])|3[01]))?|(?:\\,(?:[1-9]|(?:[12][0-9])|3[01]))*)) "
                                             "(\\*|(?:[1-9]|1[012]|JAN|FEB|MAR|APR|MAY|JUN|JUL|AUG|SEP|OCT|NOV|DEC)"  // month
                                             "(?:(?:\\-(?:[1-9]|1[012]|JAN|FEB|MAR|APR|MAY|JUN|JUL|AUG|SEP|OCT|NOV|DEC))?|"
                                             "(?:\\,(?:[1-9]|1[012]|JAN|FEB|MAR|APR|MAY|JUN|JUL|AUG|SEP|OCT|NOV|DEC))*)) "
                                             "(\\*|\\?|[0-6](?:L|\\#[1-5])?|(?:[0-6]|SUN|MON|TUE|WED|THU|FRI|SAT)"  // day_of_week
                                             "(?:(?:\\-(?:[0-6]|SUN|MON|TUE|WED|THU|FRI|SAT))?|(?:\\,(?:[0-6]|SUN|MON|TUE|WED|THU|FRI|SAT))*))"
                                             ;  // " (\\*|(?:[1-9][0-9]{3})(?:(?:\\-[1-9][0-9]{3})?|(?:\\,[1-9][0-9]{3})*))";  // year

namespace pg_rviz_plugins {

typedef enum class PCColumn {
  DEVICE = 0,
  NAME,
  ANGLE_FROM,
  ANGLE_TO,
  PUMP_DURATION,
  SCHEDULE,
  COUNT
};

QString toString(PCColumn column) {
  switch (column) {
    case PCColumn::DEVICE:
      return "Device";
    case PCColumn::NAME:
      return "Name";
    case PCColumn::ANGLE_FROM:
      return "Angle from";
    case PCColumn::ANGLE_TO:
      return "Angle to";
    case PCColumn::PUMP_DURATION:
      return "Pump duration";
    case PCColumn::SCHEDULE:
      return "Schedule";
  }
  return "UNKNOWN";
}

PlantGuardPanel::PlantGuardPanel(QWidget *parent)
    : rviz_common::Panel(parent) {
  auto layout = new QVBoxLayout;

  joystick_ = new PumpCraneJoystick;
  connect(joystick_, SIGNAL(outputJoystick(int, bool)), this, SLOT(onPumpCraneJoystick(int, bool)));
  layout->addWidget(joystick_);

  pc_jobs_table_ = new bagtube_rviz_plugins::BagtubeTableWidget;
  pc_jobs_table_->setColumnCount((int)PCColumn::COUNT);
  QStringList headers;
  for (int i = 0; i < (int)PCColumn::COUNT; i++)
    headers << toString((PCColumn)i);
  pc_jobs_table_->setHorizontalHeaderLabels(headers);
  pc_jobs_table_->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  connect(pc_jobs_table_, SIGNAL(cellClicked(int, int)), this, SLOT(onPumpCraneJobSelected(int, int)));
  connect(pc_jobs_table_, SIGNAL(deletePressed(int, int)), this, SLOT(onPumpCraneJobDelete(int, int)));
  connect(pc_jobs_table_, SIGNAL(cellChanged(int, int)), this, SLOT(onPumpCraneJobEdited(int, int)));
  layout->addWidget(pc_jobs_table_);

  auto button_layout = new QHBoxLayout;
  create_job_button_ = new QPushButton("Create job");
  connect(create_job_button_, SIGNAL(clicked()), this, SLOT(onCreateJobButtonClicked()));
  button_layout->addWidget(create_job_button_);

  capture_button_ = new QPushButton("Capture");
  capture_button_->setEnabled(false);
  connect(capture_button_, SIGNAL(clicked()), this, SLOT(onCaptureButtonClicked()));
  button_layout->addWidget(capture_button_);

  auto refresh_button = new QPushButton("Refresh");
  connect(refresh_button, SIGNAL(clicked()), this, SLOT(refreshJobs()));
  button_layout->addWidget(refresh_button);

  layout->addLayout(button_layout);

  status_label_ = new QLabel;
  layout->addWidget(status_label_);

  setLayout(layout);
}

void PlantGuardPanel::onInitialize() {
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  auto node_lock = rviz_ros_node_.lock();
  auto node = node_lock->get_raw_node();

  sync_response_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sync_response_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  sync_response_executor_->add_callback_group(sync_response_cb_group_, node->get_node_base_interface());

  pc_movement_dir_pub_ = node->create_publisher<std_msgs::msg::Int32>("/pump_crane/movement_dir_cmd", 10);
  pc_pump_pub_ = node->create_publisher<std_msgs::msg::Bool>("/pump_crane/pump_cmd", 10);
  republish_timer_ = node->create_wall_timer(100ms, [this]() {
    std::unique_lock<std::mutex> lock(stored_msgs_mutex_);
    if (dir_msg_.data != 0)
      pc_movement_dir_pub_->publish(dir_msg_);
    if (pump_msg_.data)
      pc_pump_pub_->publish(pump_msg_);
  });
  pc_angle_sub_ = node->create_subscription<pg_msgs::msg::Float64Stamped>(
      "/pump_crane/angle",
      10,
      [this](const pg_msgs::msg::Float64Stamped::SharedPtr msg) {
        std::unique_lock<std::mutex> lock(stored_msgs_mutex_);
        last_angle_ = msg->data;
        if (pc_jobs_table_->currentColumn() == (int)PCColumn::ANGLE_FROM || pc_jobs_table_->currentColumn() == (int)PCColumn::ANGLE_TO)
          capture_button_->setEnabled(true);
      }
  );
  pc_pump_sub_ = node->create_subscription<pg_msgs::msg::BoolStamped>(
      "/pump_crane/pump",
      10,
      [this](const pg_msgs::msg::BoolStamped::SharedPtr msg) {
        if (!msg->data && last_pump_time_.nanoseconds() > 0) {
          std::unique_lock<std::mutex> lock(stored_msgs_mutex_);
          last_pump_duration_ = (rclcpp::Time(msg->header.stamp) - last_pump_time_).seconds();
          last_pump_time_ = rclcpp::Time(0);
          if (pc_jobs_table_->currentColumn() == (int)PCColumn::PUMP_DURATION)
            capture_button_->setEnabled(true);
        }
        else if (msg->data && last_pump_time_.nanoseconds() == 0)
          last_pump_time_ = rclcpp::Time(msg->header.stamp);
      }
  );

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

  refreshJobs();
}

void PlantGuardPanel::setVisible(bool visible) {
  if (visible != isVisible()) {
    if (!rviz_ros_node_.lock())
      return;
    if (!visible)
      onPumpCraneJoystick(0, false);
  }

  rviz_common::Panel::setVisible(visible);
}

void PlantGuardPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void PlantGuardPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
}

template<typename MsgT>
typename MsgT::Response::SharedPtr PlantGuardPanel::callService(typename rclcpp::Client<MsgT>::SharedPtr client, const typename MsgT::Request::SharedPtr& request)
{
  bool connected = client->wait_for_service(800ms);
  if (!connected)
    return nullptr;
  auto result = client->async_send_request(request);
  typename MsgT::Response::SharedPtr response;
  if (sync_response_executor_->spin_until_future_complete(result, 800ms) == rclcpp::FutureReturnCode::SUCCESS &&
      (response = result.get())) {
    return response;
  }
  return nullptr;
}

void PlantGuardPanel::onPumpCraneJoystick(int direction, bool pump) {
  std::unique_lock<std::mutex> lock(stored_msgs_mutex_);
  dir_msg_.data = direction;
  pump_msg_.data = pump;

  pc_movement_dir_pub_->publish(dir_msg_);
  pc_pump_pub_->publish(pump_msg_);
}

void PlantGuardPanel::refreshJobs() {
  auto request = std::make_shared<pg_msgs::srv::GetJobList::Request>();
  auto response = callService<pg_msgs::srv::GetJobList>(get_jobs_cl_, request);
  if (!response) {
    RCLCPP_ERROR(LOGGER, "Failed to get job list");
    status_label_->setText("Failed to get job list");
    return;
  }

  EditingCellGuard guard(this);
  int prev_selected_row = pc_jobs_table_->currentRow();

  bool had_incomplete_job = false;
  if (pc_jobs_table_->rowCount() > 0)
    for (int j = 0; j < pc_jobs_table_->columnCount(); j++)
      if (pc_jobs_table_->item(pc_jobs_table_->rowCount() - 1, j)->text().isEmpty()) {
        had_incomplete_job = true;
        break;
      }
  if (had_incomplete_job)
    for (int i = pc_jobs_table_->rowCount() - 2; i >= 0; i--)
      pc_jobs_table_->removeRow(i);
  else
    pc_jobs_table_->setRowCount(0);

  for (int i = 0; i < response->jobs.size(); i++) {
    pc_jobs_table_->insertRow(i);
    pc_jobs_table_->setItem(i, (int)PCColumn::DEVICE, new QTableWidgetItem(QString::fromStdString(response->jobs[i].device.name)));
    pc_jobs_table_->setItem(i, (int)PCColumn::NAME, new QTableWidgetItem(QString::fromStdString(response->jobs[i].name)));
    pc_jobs_table_->setItem(i, (int)PCColumn::ANGLE_FROM, new QTableWidgetItem(QString::number(response->jobs[i].pump_crane_job.angle_from/M_PI*180.0)));
    pc_jobs_table_->setItem(i, (int)PCColumn::ANGLE_TO, new QTableWidgetItem(QString::number(response->jobs[i].pump_crane_job.angle_to/M_PI*180.0)));
    pc_jobs_table_->setItem(i, (int)PCColumn::PUMP_DURATION, new QTableWidgetItem(QString::number(response->jobs[i].pump_crane_job.pump_duration)));
    pc_jobs_table_->setItem(i, (int)PCColumn::SCHEDULE, new QTableWidgetItem(QString::fromStdString(response->jobs[i].crontab_schedule)));
  }

  if (prev_selected_row >= 0)
    pc_jobs_table_->selectRow(std::min(pc_jobs_table_->rowCount() - 1, prev_selected_row));
}

void PlantGuardPanel::onPumpCraneJobSelected(int row, int column) {
  if (row != -1)
    current_cell_text_ = pc_jobs_table_->item(row, column)->text();
  else
    pc_jobs_table_->setCurrentCell(-1, -1);
  updateButtonsAvailability();
}

bool parse_double(std::string in, double& res) {
  try {
    size_t read= 0;
    res = std::stod(in, &read);
    if (in.size() != read)
      return false;
  } catch (std::invalid_argument) {
    return false;
  }
  return true;
}

bool PlantGuardPanel::isJobCellValid(int row, int column, std::string *error_msg) {
  std::string text = pc_jobs_table_->item(row, column)->text().toStdString();
  if (text.empty())
    return false;
  switch ((PCColumn)column) {
    case PCColumn::DEVICE:
      if (text != "pump_crane") {  // for now, only pump_crane is supported
        if (error_msg)
          *error_msg = "Invalid device (only pump_crane is supported for now)";
        return false;
      }
      return true;
    case PCColumn::NAME: {
      if (!std::regex_match(text, std::regex(ALLOWED_NAMING_PATTERN, std::regex_constants::basic))) {
        if (error_msg)
          *error_msg = "Invalid name";
        return false;
      }
      bool unique = true;
      for (int i = 0; i < pc_jobs_table_->rowCount(); i++) {
        if (i == row)
          continue;
        if (pc_jobs_table_->item(i, (int)PCColumn::NAME)->text().toStdString() == text) {
          unique = false;
          if (error_msg)
            *error_msg = "Name must be unique";
          break;
        }
      }
      return unique;
    }
    case PCColumn::ANGLE_FROM:
    case PCColumn::ANGLE_TO:
    case PCColumn::PUMP_DURATION: {
      if (double value; parse_double(text, value) && ((PCColumn)column != PCColumn::PUMP_DURATION || value >= 0))
        return true;
      else {
        if (error_msg)
          *error_msg = "Invalid number";
        return false;
      }
    }
    case PCColumn::SCHEDULE: {
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

void PlantGuardPanel::onPumpCraneJobEdited(int row, int column) {
  if (editing_cells_internally_)
    return;
  EditingCellGuard guard(this);
  std::string error_msg;
  if (!isJobCellValid(row, column, &error_msg)) {
    status_label_->setText("Invalid value in " + toString((PCColumn)column) + " column: " + QString::fromStdString(error_msg));
    pc_jobs_table_->setItem(row, column, new QTableWidgetItem(current_cell_text_));
    editing_cells_internally_ = false;
    return;
  }

  bool is_complete_row = true;
  for (int i = 0; i < pc_jobs_table_->columnCount(); i++)
    if (!pc_jobs_table_->item(row, i) || pc_jobs_table_->item(row, i)->text().isEmpty()) {
      is_complete_row = false;
      break;
    }
  if (!is_complete_row)
    return;

  auto request = std::make_shared<pg_msgs::srv::EditJob::Request>();
  request->command = pg_msgs::srv::EditJob::Request::ADD_OR_MODIFY;
  request->job.device.name = pc_jobs_table_->item(row, (int)PCColumn::DEVICE)->text().toStdString();
  request->job.device.type = pg_msgs::msg::Device::TYPE_PUMP_CRANE;
  request->job.name = pc_jobs_table_->item(row, (int)PCColumn::NAME)->text().toStdString();
  request->job.pump_crane_job.angle_from = std::stod(pc_jobs_table_->item(row, (int)PCColumn::ANGLE_FROM)->text().toStdString())/180.0*M_PI;
  request->job.pump_crane_job.angle_to = std::stod(pc_jobs_table_->item(row, (int)PCColumn::ANGLE_TO)->text().toStdString())/180.0*M_PI;
  request->job.pump_crane_job.pump_duration = std::stod(pc_jobs_table_->item(row, (int)PCColumn::PUMP_DURATION)->text().toStdString());
  request->job.crontab_schedule = pc_jobs_table_->item(row, (int)PCColumn::SCHEDULE)->text().toStdString();

  auto response = callService<pg_msgs::srv::EditJob>(edit_job_cl_, request);
  if (!response || !response->success) {
    RCLCPP_ERROR(LOGGER, "Failed to edit job %s: %s", request->job.name.c_str(), response ? response->message.c_str() : "no response");
    status_label_->setText("Failed to edit job " + QString::fromStdString(request->job.name) + ": " + QString::fromStdString(response ? response->message : "no response"));
    pc_jobs_table_->setItem(row, column, new QTableWidgetItem(current_cell_text_));
    return;
  }
  status_label_->setText("Job " + QString::fromStdString(request->job.name) + " edited");
  updateButtonsAvailability();
}

void PlantGuardPanel::onPumpCraneJobDelete(int row, int /*column*/) {
  bool deleting_complete_row = true;
  for (int i = 0; i < pc_jobs_table_->columnCount(); i++)
    if (pc_jobs_table_->item(row, i)->text().isEmpty()) {
      deleting_complete_row = false;
      break;
    }
  if (deleting_complete_row) {
    RCLCPP_INFO(LOGGER, "Deleting job %s", pc_jobs_table_->item(row, (int)PCColumn::NAME)->text().toStdString().c_str());
    auto request = std::make_shared<pg_msgs::srv::EditJob::Request>();
    request->command = pg_msgs::srv::EditJob::Request::DELETE;
    request->job.device.name = pc_jobs_table_->item(row, (int)PCColumn::DEVICE)->text().toStdString();
    request->job.name = pc_jobs_table_->item(row, (int)PCColumn::NAME)->text().toStdString();

    auto response = callService<pg_msgs::srv::EditJob>(edit_job_cl_, request);
    if (!response) {
      RCLCPP_ERROR(LOGGER, "Failed to delete job %s", request->job.name.c_str());
      status_label_->setText("Failed to delete job " + QString::fromStdString(request->job.name));
      return;
    }
  }
  pc_jobs_table_->removeRow(row);
  updateButtonsAvailability();
}

void PlantGuardPanel::updateButtonsAvailability() {
  bool can_create = true;
  for (int i = 0; i < pc_jobs_table_->rowCount(); i++) {
    for (int j = 0; j < pc_jobs_table_->columnCount(); j++)
      if (pc_jobs_table_->item(i, j)->text().isEmpty()) {
        can_create = false;
        break;
      }
    if (!can_create)
      break;
  }
  create_job_button_->setEnabled(can_create);

  auto column = (PCColumn)pc_jobs_table_->currentColumn();
  bool can_capture =
      ((column == PCColumn::ANGLE_FROM || column == PCColumn::ANGLE_TO) && std::isfinite(last_angle_)) ||
      (column == PCColumn::PUMP_DURATION && std::isfinite(last_pump_duration_));
  capture_button_->setEnabled(can_capture);
}

void PlantGuardPanel::onCreateJobButtonClicked() {
  EditingCellGuard guard(this);
  pc_jobs_table_->insertRow(pc_jobs_table_->rowCount());
  pc_jobs_table_->setItem(pc_jobs_table_->rowCount() - 1, (int)PCColumn::DEVICE, new QTableWidgetItem("pump_crane"));

  int job_number = pc_jobs_table_->rowCount();
  bool unique;
  do {
    unique = true;
    for (int i = 0; i < pc_jobs_table_->rowCount() - 1; i++) {
      if (pc_jobs_table_->item(i, (int)PCColumn::NAME)->text().toStdString() == "job" + std::to_string(job_number)) {
        job_number++;
        unique = false;
        break;
      }
    }
  } while (!unique);
  pc_jobs_table_->setItem(pc_jobs_table_->rowCount() - 1, (int)PCColumn::NAME, new QTableWidgetItem("job" + QString::number(job_number)));

  for (int i = (int)PCColumn::NAME + 1; i < (int)PCColumn::COUNT; i++)
    pc_jobs_table_->setItem(pc_jobs_table_->rowCount() - 1, i, new QTableWidgetItem(""));

  create_job_button_->setEnabled(false);
}

void PlantGuardPanel::onCaptureButtonClicked() {
  auto column = (PCColumn)pc_jobs_table_->currentColumn();
  if (column == PCColumn::ANGLE_FROM || column == PCColumn::ANGLE_TO) {
    pc_jobs_table_->item(pc_jobs_table_->currentRow(), (int)column)->setText(QString::number(last_angle_/M_PI*180.0));
  }
  else if (column == PCColumn::PUMP_DURATION) {
    pc_jobs_table_->item(pc_jobs_table_->currentRow(), (int)column)->setText(QString::number(last_pump_duration_));
  }
}

} // pg_rviz_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(pg_rviz_plugins::PlantGuardPanel, rviz_common::Panel)
