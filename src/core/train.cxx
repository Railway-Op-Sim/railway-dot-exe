#include "railos/train.hxx"
#include "railos/logging.hxx"
#include <string>

namespace RailOS::Train {

int next_train_id{0};

Train::Train(int caller, int rear_start_elem_in, int rear_start_exit_pos_in,
             const std::string &input_code, int start_speed, int mass,
             double max_running_speed, double max_brake_rate,
             double power_at_rail, Mode train_mode,
             std::shared_ptr<Actions::TrainDataEntry> &train_data_entry,
             int repeat_number, int incremental_mins, int incremental_digits,
             int signaller_max_speed)
    : rear_start_element(rear_start_elem_in),
      rear_start_exit_pos(rear_start_exit_pos_in), head_code(input_code),
      start_speed(start_speed), mass(mass),
      max_running_speed(max_running_speed), max_brake_rate(max_brake_rate),
      power_at_rail(power_at_rail), train_mode(train_mode),
      train_data_entry(train_data_entry), repeat_number(repeat_number),
      incremental_mins(incremental_mins),
      incremental_digits(incremental_digits),
      signaller_max_speed(signaller_max_speed),
      timetable_max_running_speed(max_running_speed),
      a_value(std::sqrt(2 * power_at_rail / mass)),
      train_id(next_train_id),
      original_power_at_rail(power_at_rail)
/*
              Construct a new train with general default values and input values
   for position and headcode. Create the frontcode, headcode and background
   graphics here but don't delete them in a destructor. This is because trains
   are kept in a vector and vectors erase elements during internal operations.
              Deletion is explicit by using a special function.  Increment the
   static class member NextTrainID after setting this train's ID.
*/
{
  Logging::call_log.push_back(
      Utilities::getTimeStamp() + "," + std::to_string(caller) + ",TTrain," +
      std::to_string(rear_start_elem_in) + "," +
      std::to_string(rear_start_exit_pos_in) + "," + input_code + "," +
      std::to_string(start_speed) + "," + std::to_string(mass) + "," +
      std::to_string(static_cast<int>(train_mode)));

  next_train_id++;
  Logging::popCallLog(648);
}

}; // namespace RailOS::Train