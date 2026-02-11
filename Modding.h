//---------------------------------------------------------------------------

#ifndef ModdingH
#define ModdingH

#include <Graphics.hpp>

#include <optional>
#include <memory>
#include <vector>
#include <filesystem>
//---------------------------------------------------------------------------
// Introduce concept of "Modding", users can replace content in Mods directory

extern std::string RESOURCE_PREFIX;
extern const std::string MODS_DIRECTORY;

class RuntimeModifier {
	private:
		std::optional<std::string> graphics_library_{std::nullopt};
		const std::string mods_directory_{"Mods"};
		std::vector<std::function<void()>> callbacks_;
		void triggerCallbacks_() const {
			for(auto& callback : callbacks_) {
                callback();
            }
        }
    public:

		void attachCallback(std::function<void()> callback) {
            // Call once for initial setup
			callback();

			// Now attach to tracked callbacks, these will be executed
            // if the modifier is changed
            callbacks_.push_back(std::move(callback));
		}

		void setGraphicsLibrary(const std::string& graphics_prefix) {
			graphics_library_ = std::optional<std::string>{graphics_prefix};
            triggerCallbacks_();
        }

		// Returns the current mod graphics directory, if applicable
		std::optional<std::string> getCurrentGraphicsDirectory() const {
			if (!graphics_library_.has_value()) {
				 return std::nullopt;
			}
			return mods_directory_ + "\\" + graphics_library_.value();
		}

		void loadGraphic(TBitmap* target, const std::string& graphic) {
			const std::optional<std::string> current_graphics_dir{getCurrentGraphicsDirectory()};

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
				} catch (const System::Sysutils::Exception &e) {}
			}

			target->LoadFromResourceName(0, graphic.c_str());
		}
};

extern std::unique_ptr<RuntimeModifier> Modifier;
#endif
