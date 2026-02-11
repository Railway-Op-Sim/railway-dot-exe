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
        const std::string library_icon_default_{"Library"};

		std::vector<std::function<void()>> callbacks_;

		void trigger_callbacks_() const {
			for(auto& callback : callbacks_) {
                callback();
            }
		}

		void create_directories_() const;
	public:
		RuntimeModifier() {
            create_directories_();
		}

		std::optional<std::string> get_current_graphics_library() const {
            return graphics_library_;
        }

		void attach_callback(std::function<void()> callback);

		std::vector<std::string> get_graphics_libraries() const;

		void set_graphics_library(const std::optional<std::string>& graphics_prefix);

		std::optional<std::string> get_library_icon_file(const std::string& library) const;

		// Returns the current mod graphics directory, if applicable
		std::optional<std::string> get_current_graphics_directory() const;

		void load_graphic(TBitmap* target, const std::string& graphic);
};

extern std::unique_ptr<RuntimeModifier> Modifier;
#endif
