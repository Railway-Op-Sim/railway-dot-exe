//---------------------------------------------------------------------------

#pragma hdrstop

#include "Modding.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
std::unique_ptr<RuntimeModifier> Modifier{new RuntimeModifier};

void RuntimeModifier::create_directories_() const {

	std::vector<std::string> sub_directories_{
		"Graphics",
	};

	for(const std::string& dir : sub_directories_) {
		try {
			std::filesystem::create_directories(mods_directory_ + "/" + dir);
		} catch (const std::filesystem::filesystem_error& e) {
		   std::cerr << "RuntimeModifier Error: " << e.what() << "\n";
		}
	}
}


void RuntimeModifier::attach_callback(std::function<void()> callback) {
	// Call once for initial setup
	callback();

	// Now attach to tracked callbacks, these will be executed
	// if the modifier is changed
	callbacks_.push_back(std::move(callback));
}

std::vector<std::string> RuntimeModifier::get_graphics_libraries() const {
	std::vector<std::string> graphics_dirs_;
	for(const auto& addr : std::filesystem::directory_iterator(mods_directory_ + "\\Graphics")) {
		if(!std::filesystem::is_directory(addr.status())) continue;
		graphics_dirs_.push_back(addr.path().filename().generic_string());
	}
	return graphics_dirs_;
}

void RuntimeModifier::set_graphics_library(const std::optional<std::string>& graphics_prefix) {
	graphics_library_ = graphics_prefix;
	trigger_callbacks_();
}

// Returns the current mod graphics directory, if applicable
std::optional<std::string> RuntimeModifier::get_current_graphics_directory() const {
	if (!graphics_library_.has_value()) {
		 return std::nullopt;
	}
	return mods_directory_ + "\\Graphics\\" + graphics_library_.value();
}

void RuntimeModifier::apply_color_scheme_(TBitmap* target) {
    // Graphics should be defined for a white background by default
	for(int y{0}; y < target->Height; ++y) {
		for(int x{0}; x < target->Width; ++y) {
			const TColor current_pixel_ = target->Canvas->Pixels[x][y];
            TColor new_color_ = current_pixel_;

			if(background_color_ != clWhite) {
				if(current_pixel_ == background_color_) {
					new_color_ = foreground_color_;
				} else if (current_pixel_ == clBlack) {
                    new_color_ = background_color_;
				}
			}

            target->Canvas->Pixels[x][y] = new_color_;
        }
    }
}

void RuntimeModifier::load_graphic(TBitmap* target, const std::string& graphic, const Transparency transparency) {
	const std::optional<std::string> current_graphics_dir{get_current_graphics_directory()};

    std::optional<std::string> local_file_ = std::nullopt;

	if(graphics_library_.has_value()) {
		std::string graphic_{graphic};
		std::transform(graphic_.begin(), graphic_.end(), graphic_.begin(),
		   [](unsigned char c){ return std::toupper(c);});
		local_file_ = std::optional<std::string>(
			current_graphics_dir.value() + "\\" + graphic_ + ".bmp"
		);
	}

	if(local_file_.has_value() && std::filesystem::exists(local_file_.value())) {
		target->LoadFromFile(local_file_.value().c_str());
	} else {
		target->LoadFromResourceName(0, graphic.c_str());
	}

	if(transparency != Transparency::Undefined) {
		target->Transparent = transparency == Transparency::Transparent;
		target->TransparentColor = clWhite;
    }
}

std::optional<std::string> RuntimeModifier::get_library_icon_file(const std::string& library) const {
	const std::filesystem::path target_dir_{mods_directory_ + "/Graphics/" + library};
	if(!std::filesystem::exists(target_dir_) || !std::filesystem::is_directory(target_dir_)) {
		return std::nullopt;
	}
	const std::filesystem::path default_icon_ = target_dir_ / std::filesystem::path(library_icon_default_ + ".bmp");

	if(std::filesystem::exists(default_icon_)) return default_icon_.string();

	for (const auto& entry : std::filesystem::directory_iterator(target_dir_)) {
		if (!std::filesystem::is_regular_file(entry)) continue;

		std::string ext = entry.path().extension().string();
		for (auto& c : ext) c = tolower(c);

		if (ext == ".bmp") {
			return entry.path().string(); // first .bmp found
		}
	}
	return std::nullopt;
}
