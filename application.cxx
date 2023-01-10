#include "nana/basic_types.hpp"
#include <nana/gui.hpp>
#include <iostream>

#include <nana/gui/widgets/button.hpp>
#include <nana/gui/widgets/label.hpp>
#include "railos/ui/buttons.hxx"

#ifndef RAILOS_APPLICATION_NAME
#error Macro RAILOS_APPLICATION_NAME must be defined!
#endif

#ifndef RAILOS_VERSION
#error Macro RAILOS_VERSION must be defined!
#endif


int main() {
    auto monitor_size_{nana::screen::primary_monitor_size()};
    std::shared_ptr<nana::form> fm_{ new nana::form{ nana::rectangle{ 10, 10, monitor_size_.width, monitor_size_.height} } };
    nana::place place_{ fm_->handle() };
    fm_->caption(std::string(RAILOS_APPLICATION_NAME) + std::string(" ") + std::string(RAILOS_VERSION));
    std::unique_ptr<nana::button> button_ = RailOS::UI::create_track_button_(fm_, "track_piece_1.png");
    button_->move(20, 20);
    place_.div("<button>");
    place_["button"] << button_->handle();
    fm_->show();
    nana::exec();
}