//---------------------------------------------------------------------------

#include "metadata/reader.hxx"
#include "metadata/toml.hxx"
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>

SessionMetadata MetadataReader::read_metadata_from_file(const std::filesystem::path& metadata_file, bool verbose) {
	
	const std::filesystem::path outfile_prefix_{output_directory_ / metadata_file.stem()};
	const std::filesystem::path outfile_{outfile_prefix_.string() + ".log"};

	ValidationResult toml_parse_;

	try {
		reader_.read(metadata_file, verbose);
	} catch(TOMLException& e) {
		ValidationResult validation_;
		toml_parse_[ValidationError::ValueError].push_back(e.what());
		toml_parse_.dump(outfile_);
	}
	
	const ValidationResult validation_{validate()};

	if(!validation_.empty() || !toml_parse_.empty()) {
		if(!toml_parse_.empty()) {
			toml_parse_.dump(outfile_);	
		} else {
			validation_.dump(outfile_);
		}
		throw std::invalid_argument(
			std::string("Validation of session metadata failed. ") +
			"Result written to '" +
			outfile_.string() +
			"'."
		);
	}

	const SessionMetadata session_data_{
		reader_.get_string("name").value(),
		reader_.get_string("description"),
		reader_.get_string("display_name"),
		reader_.get_string("rly_file").value(),
		reader_.get_list("ttb_files").value(),
		reader_.get_list("ssn_files"),
		reader_.get_list("doc_files").value(),
		reader_.get_list("img_files"),
		reader_.get_list("graphics_files"),
		iso::from_string(reader_.get_string("country_code").value()),
		reader_.get_integer("year"),
		static_cast<bool>(reader_.get_boolean("factual")),
		reader_.get_integer("difficulty"),
		reader_.get_string("author").value(),
		reader_.get_list("contributors"),
		reader_.get_version("version").value(),
		reader_.get_date("release_date").value(),
		reader_.get_version("minimum_required"),
		(reader_.get_string("signal_position").has_value()) ? ((reader_.get_string("signal_position").value() == "left") ? SignalPosition::Left : SignalPosition::Right) : SignalPosition::Left
	};
	return session_data_;
}

ValidationResult MetadataReader::validate() const {
	ValidationResult result_;

	// Check for missing keys
	for(auto [key, _] : required_types_) {
		if(!reader_.has_key(key)) {
			result_[ValidationError::MissingKey].push_back(key);
		}
	}

	// Check types
	for(auto [key, type] : required_types_) {
		bool failed_validation_{false};
		switch (type) {
			case Type::Integer:
				failed_validation_ = !reader_.get_integer(key).has_value();
				break;
			case Type::String:
				failed_validation_ = !reader_.get_string(key).has_value();
				break;
			case Type::Boolean:
				failed_validation_ = !reader_.get_boolean(key).has_value();
				break;
			case Type::List:
				failed_validation_ = !reader_.get_list(key).has_value();
				break;
			case Type::Version:
				failed_validation_ = !reader_.get_version(key).has_value();
				break;
			case Type::Date:
				failed_validation_ = !reader_.get_date(key).has_value();
				break;
		}
		if(failed_validation_) {
			result_[ValidationError::IncorrectType].push_back(key);
		}
	}

	for(auto [key, type] : optional_types_) {
		if(!reader_.has_key(key)) continue;
		bool failed_validation_{false};
		switch (type) {
			case Type::Integer:
				failed_validation_ = !reader_.get_integer(key).has_value();
				break;
			case Type::String:
				failed_validation_ = !reader_.get_string(key).has_value();
				break;
			case Type::Boolean:
				failed_validation_ = !reader_.get_boolean(key).has_value();
				break;
			case Type::List:
				failed_validation_ = !reader_.get_list(key).has_value();
				break;
			case Type::Version:
				failed_validation_ = !reader_.get_version(key).has_value();
				break;
			case Type::Date:
				failed_validation_ = !reader_.get_date(key).has_value();
				break;
		}
		if(failed_validation_) {
			result_[ValidationError::IncorrectType].push_back(key);
		}
	}
	return result_;
}

void ValidationResult::dump(const std::filesystem::path& output_file) const {
	const std::filesystem::path directory_{output_file.parent_path()};
	if(!std::filesystem::exists(directory_)) {
		throw std::invalid_argument(
			"Cannot write metadata validation result, directory '" +
			directory_.string() +
			"' directory does not exist."
		);
	}
	std::ofstream output_{output_file};
	output_ << (*this) << std::endl;
	output_.close();
}

void MetadataReader::read_local_simulation(const std::string& simulation_name) {

}
