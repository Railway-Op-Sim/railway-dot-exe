#include <nana/gui.hpp>
#include <nana/gui/widgets/button.hpp>

#include <filesystem>

#ifndef RAILOS_GRAPHICS_ROOT
#error Macro RAILOS_GRAPHICS_ROOT must be defined!
#endif

namespace RailOS::UI {
    const std::filesystem::path GRAPHICS_ROOT_{RAILOS_GRAPHICS_ROOT};
    std::unique_ptr<nana::button> create_track_button_(nana::form form, std::filesystem::path image_url);
};