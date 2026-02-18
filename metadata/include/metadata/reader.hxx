//---------------------------------------------------------------------------

#pragma once
//---------------------------------------------------------------------------

#include <fstream>
#include <optional>
#include <ostream>
#include <vector>
#include <filesystem>
#include <string>
#include <map>
#include <regex>
#include <iostream>
#include <stdexcept>

#include "metadata/iso_country_codes.hxx"
#include "metadata/toml.hxx"

enum class ValidationError {
	MissingKey,
	IncorrectType,
    Empty,
	ValueError,
	FileNotFound
};

class MetadataFileException : public std::invalid_argument {
	public:
		explicit MetadataFileException(const std::string& message) : std::invalid_argument(message) {}
};

class ValidationResult : public std::map<ValidationError, std::vector<std::string>> {
	public:
		ValidationResult() {
			this->insert({ValidationError::MissingKey, {}});
			this->insert({ValidationError::IncorrectType, {}});
            this->insert({ValidationError::Empty, {}});
			this->insert({ValidationError::ValueError, {}});
			this->insert({ValidationError::FileNotFound, {}});
		}
		ValidationResult(ValidationResult& other) {
			(*this) = other;
		}
		bool empty() const {
			return (
				this->at(ValidationError::MissingKey).empty() &&
				this->at(ValidationError::IncorrectType).empty() &&
                this->at(ValidationError::Empty).empty() &&
				this->at(ValidationError::ValueError).empty() &&
				this->at(ValidationError::FileNotFound).empty()
			);
		}
		void dump(const std::filesystem::path& output_file) const;
		friend std::ostream& operator<<(std::ostream& os, const ValidationResult& validation) {
			for(auto entry : validation.at(ValidationError::ValueError)) {
				os << "VALUE_ERROR: " << entry << std::endl;
			}
			for(auto entry : validation.at(ValidationError::MissingKey)) {
				os << "MISSING_KEY: Mandatory key '" << entry << "' is missing." << std::endl;
			}
            for(auto entry : validation.at(ValidationError::Empty)) {
				os << "EMPTY: Value for '" << entry << "' is an empty list." << std::endl;
			}
			for(auto entry : validation.at(ValidationError::IncorrectType)) {
				os << "INVALID_FORMAT: Value for '" << entry << "' is in invalid format." << std::endl;
			}
			return os;
		}
};

enum class SignalPosition {
	Left,
	Right
};

struct SimulationMetadata {
	std::string name{""};
	std::optional<std::string> description{std::nullopt};
	std::optional<std::string> display_name{std::nullopt};
	std::string rly_file{""};
	std::vector<std::string> ttb_files;
	std::optional<std::vector<std::string>> ssn_files{std::nullopt};
	std::vector<std::string> doc_files;
	std::optional<std::vector<std::string>> img_files{std::nullopt};
	std::optional<std::vector<std::string>> graphics_files{std::nullopt};
	std::string country_code{"FN"};
	std::optional<unsigned int> year{std::nullopt};
	bool factual{true};
	std::optional<unsigned int> difficulty{std::nullopt};
	std::string author{""};
	std::optional<std::vector<std::string>> contributors;
	toml::Version version{0, 0, 0};
	toml::Date release_date{0, 0, 0};
	std::optional<toml::Version> minimum_required{std::nullopt};
	SignalPosition signal_position{SignalPosition::Left};

	friend std::ostream& operator<<(std::ostream& os, const SimulationMetadata& metadata) {
		os << "SimulationMetadata{";
		os << "\tname = \"" << metadata.name << "\"\n";
		os << "\tdescription = \"" << ((metadata.description.has_value()) ? (std::string("\"") + metadata.description.value() + "\"") : "null") << "\n";
		os << "\tdisplay_name = " << ((metadata.display_name.has_value()) ? (std::string("\"") + metadata.display_name.value() + std::string("\"")) : "null") << "\n";
		os << "\trly_file = \"" << metadata.rly_file << "',\n";
		os << "\tttb_files = [\n";
		for(auto ttb_file : metadata.ttb_files) {
			os << "\t\t\"" << ttb_file << "\",\n";
		}
		os << "\t],\n";
		if(metadata.ssn_files.has_value()) {
			os << "\tssn_files = [\n";
			for(auto ssn_file : metadata.ssn_files.value()) {
				os << "\t\t\"" << ssn_file << "\",\n";
			}
			os << "\t],\n";
		} else {
			os << "\tssn_files = null\n";
		}
		os << "\tdoc_files = [\n";
		for(auto doc_file : metadata.doc_files) {
			os << "\t\t\"" << doc_file << "\",\n";
		}
		os << "\t],\n";
		if(metadata.img_files.has_value()) {
			os << "\timg_files = [\n";
			for(auto img_file : metadata.img_files.value()) {
				os << "\t\t\"" << img_file << "\",\n";
			}
			os << "\t],\n";
		} else {
			os << "\timg_files = null\n";
		}
		if(metadata.graphics_files.has_value()) {
			os << "\tgraphics_files = [\n";
			for(auto graphics_file : metadata.graphics_files.value()) {
				os << "\t\t'" << graphics_file << "',\n";
			}
			os << "\t],\n";
		} else {
			os << "\tgraphics_files = null\n";
		}
		os << "\tcountry_code = \"" << metadata.country_code << "\",\n";
		os << "\tyear = " << ((metadata.year.has_value()) ? std::to_string(metadata.year.value()) : "null") << ",\n";
		os << "\tfactual = " << metadata.factual << ",\n";
		os << "\tdifficulty = " << ((metadata.difficulty.has_value()) ? std::to_string(metadata.difficulty.value()) : "null") << ",\n";
		os << "\tauthor = \"" << metadata.author << "\"\n";
		if(metadata.contributors.has_value()) {
			os << "\tcontributors = [\n";
			for(auto graphics_file : metadata.contributors.value()) {
				os << "\t\t\"" << graphics_file << "\"\n";
			}
			os << "\t]\n";
		} else {
			os << "\tcontributors = null\n";
		}
		os << "\tversion = \"";
		os << "'" << metadata.version.major;
		os << "." << metadata.version.minor;
		os << "." << metadata.version.patch;
		os << "\"";
		os << "\n";
		os << "\trelease_date = ";
		os << "\"" << metadata.release_date.year;
		os << "." << metadata.release_date.month;
		os << "." << metadata.release_date.day;
		os << "\"\n";
		os << "\tminimum_required = '";
		if(metadata.minimum_required.has_value()) {
			os << "\"" << metadata.minimum_required.value().major;
			os << "." << metadata.minimum_required.value().minor;
			os << "." << metadata.minimum_required.value().patch;
			os << "\"\n";
		} else {
			os << "null\n";
		}
		os << ((metadata.signal_position == SignalPosition::Left) ? "left" : "right");
		os << "\n";
		os << "}";
		return os;
	}
};


class MetadataReader {
	private:
    	toml::TOML reader_;
		const std::filesystem::path output_directory_;
		ValidationResult validation_;
		const std::map<std::string, toml::Type> required_types_{
			{"name", toml::Type::String},
			{"rly_file", toml::Type::String},
			{"ttb_files", toml::Type::List},
			{"doc_files", toml::Type::List},
			{"country_code", toml::Type::String},
			{"year", toml::Type::Integer},
			{"factual", toml::Type::Boolean},
			{"author", toml::Type::String},
			{"version", toml::Type::Version},
			{"release_date", toml::Type::Date},
		};
		const std::map<std::string, toml::Type> optional_types_{
			{"display_name", toml::Type::String},
			{"description", toml::Type::String},
			{"ssn_files", toml::Type::List},
			{"img_files", toml::Type::List},
			{"graphic_files", toml::Type::List},
			{"difficulty", toml::Type::Integer},
			{"contributors", toml::Type::List},
			{"minimum_required", toml::Type::Version},
			{"signal_position", toml::Type::String},
	
		};
		static std::vector<std::string> get_list_from_delimited(
			const std::string& value,
			const char delimiter=';'
		);
		std::map<std::string, SimulationMetadata> current_metadata_;
		std::map<std::string, std::string> file_directories_;
	public:
		MetadataReader(
            const std::map<std::string, std::string> file_directories,
			const std::filesystem::path& output_dir
		) : output_directory_(output_dir), file_directories_(file_directories) {}
		ValidationResult validate() const;
		void clear() {reader_.clear();}
		template<typename T>
		void insert(const std::string& key, const T& value) {
			reader_.insert<T>(key, value);
		}
		void insert_list(const std::string& key, const std::string& value) {
            const std::vector<std::string> tokens_{get_list_from_delimited(value)};
			reader_.insert<std::vector<std::string>>(key, tokens_);
        }
		SimulationMetadata read_metadata_from_file(const std::filesystem::path& metadata_file, bool verbose = false);
		std::map<std::string, SimulationMetadata> get_metadata() const {
			return current_metadata_;
		}
		void read_directory(const std::filesystem::path& directory, bool raise_except = true);
        std::optional<SimulationMetadata> get_by_prefix(const std::string& prefix) const;
};
