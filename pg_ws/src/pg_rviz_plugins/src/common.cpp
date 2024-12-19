//
// Created by matej on 11/12/24.
//

#include "pg_rviz_plugins/common.hpp"

namespace pg_rviz_plugins {

const std::string ALLOWED_NAMING_PATTERN = "(?!^__)([a-zA-Z_][a-zA-Z0-9_]*)";
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

bool parse_double(const std::string &in, double& res, double default_value) {
  try {
    size_t read= 0;
    res = std::stod(in, &read);
    if (in.size() != read) {
      res = default_value;
      return false;
    }
  } catch (std::invalid_argument &) {
    res = default_value;
    return false;
  }
  return true;
}


}