#include "railos/ui/buttons.hxx"
#include "nana/basic_types.hpp"
#include "nana/gui/element.hpp"

#include <filesystem>
#include <iostream>
#include <string>
#include <stdexcept>

namespace RailOS::UI {
    std::unique_ptr<nana::button> create_track_button_(std::shared_ptr<nana::form> form, std::filesystem::path image_url) {
        const std::filesystem::path image_path_{RAILOS_GRAPHICS_ROOT / image_url};
        if(!std::filesystem::exists(image_path_)) {
            throw std::runtime_error(std::string{"Cannot find file '"} + std::string(image_path_.u8string()) + std::string{"'"});
        }
        nana::element::bground bkg_;
        bkg_.image(nana::paint::image{image_path_.c_str()}, true, {});
        bkg_.stretch_parts(4, 4, 4, 4);
        auto button_ = std::make_unique<nana::button>(form->handle(), nana::rectangle{20, 20, 40, 40});
        button_->set_bground(bkg_);
        return button_;
    }
};