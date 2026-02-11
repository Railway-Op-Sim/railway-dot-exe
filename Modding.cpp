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
	return mods_directory_ + "\\" + graphics_library_.value();
}

void RuntimeModifier::load_graphic(TBitmap* target, const std::string& graphic) {
	const std::optional<std::string> current_graphics_dir{get_current_graphics_directory()};

	if(graphics_library_.has_value()) {
		// Let's be safe, if the graphic fails to load revert
		// back to the builtin variant
		try {
			const std::string res_file_ = (
				mods_directory_ + "\\" + "Graphics" +
				current_graphics_dir.value() + "\\" + graphic
			);

			if(std::filesystem::exists(res_file_)) {
				target->LoadFromFile(res_file_.c_str());
				return;
			}
		} catch (const System::Sysutils::Exception &e) {
            std::cerr << "RuntimeModifier Error: " << e.Message.c_str() << "\n";
		}
	}

	target->LoadFromResourceName(0, graphic.c_str());
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
