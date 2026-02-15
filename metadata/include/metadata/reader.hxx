//---------------------------------------------------------------------------

#pragma once
//---------------------------------------------------------------------------

#include <fstream>
#include <optional>
#include <vector>
#include <filesystem>
#include <string>
#include <map>
#include <regex>

#include "metadata/iso_country_codes.hxx"
#include "metadata/toml.hxx"

enum class SignalPosition {
	Left,
	Right
};


struct SessionMetadata {
	std::string name{""};
	std::optional<std::string> description{std::nullopt};
	std::optional<std::string> display_name{std::nullopt};
	std::string rly_file{""};
	std::vector<std::string> ttb_files;
	std::optional<std::vector<std::string>> ssn_files{std::nullopt};
	std::string doc_file{""};
	std::optional<std::vector<std::string>> img_files{std::nullopt};
	std::optional<std::vector<std::string>> graphics_files{std::nullopt};
	iso::Country country_code{iso::Country::Fictional};
	std::optional<unsigned int> year{std::nullopt};
	bool factual{true};
	std::optional<unsigned int> difficulty{std::nullopt};
	std::string author{""};
	std::optional<std::vector<std::string>> contributors;
	Version version;
	Date release_date;
	std::optional<Version> minimum_required{std::nullopt};
	SignalPosition signal_position{SignalPosition::Left};
};

class MetadataReader {
	private:
    	TOML reader_;
		std::optional<std::string> current_metadata_file_{std::nullopt};
        std::optional<SessionMetadata> current_metadata_{std::nullopt};
	public:
		SessionMetadata read_metadata_from_file(const std::filesystem::path& metadata_file);
        void read_local_simulation(const std::string& simulation_name);
		std::optional<SessionMetadata> get_metadata() const {
			return current_metadata_;
		}
};

extern MetadataReader Metadata;
