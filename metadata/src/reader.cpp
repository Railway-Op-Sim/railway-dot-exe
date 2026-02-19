//---------------------------------------------------------------------------

#include "metadata/reader.hxx"
#include "metadata/toml.hxx"
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>

SimulationMetadata MetadataReader::read_metadata_from_file(const std::filesystem::path& metadata_file, bool verbose) {
	
	const std::filesystem::path outfile_prefix_{output_directory_ / metadata_file.stem()};
	const std::filesystem::path outfile_{outfile_prefix_.string() + ".log"};

	ValidationResult toml_parse_;

	try {
		reader_.load(metadata_file, verbose);
	} catch(toml::TOMLException& e) {
		ValidationResult validation_;
		toml_parse_[ValidationError::ValueError].insert(e.what());
		toml_parse_.dump(outfile_);
	}
	
	const ValidationResult validation_{validate()};

	if(!validation_.empty() || !toml_parse_.empty()) {
		if(!toml_parse_.empty()) {
			toml_parse_.dump(outfile_);	
		} else {
			validation_.dump(outfile_);
		}
		throw MetadataFileException(
			std::string("Validation of 'Metadata\\") +
			metadata_file.filename().string() +
			"' failed. " +
			"Result written to '" +
			outfile_.string() +
			"'."
		);
	}

	const SimulationMetadata session_data_{
		reader_.get_string("name").value(),
		reader_.get_string("description"),
		reader_.get_string("display_name"),
		reader_.get_string("rly_file").value(),
		reader_.get_list("ttb_files").value(),
		reader_.get_list("ssn_files"),
		reader_.get_list("doc_files").value(),
		reader_.get_list("img_files"),
		reader_.get_list("graphics_files"),
		reader_.get_string("country_code").value(),
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
			result_[ValidationError::MissingKey].insert(key);
		}
	}

	const std::string country_code_{reader_.get_string("country_code").value()};

	const bool valid_cc_{std::any_of(
		iso::country_codes.begin(),
		iso::country_codes.end(),
		[&](const auto& pair){ return pair.second == country_code_; })
	};

	if(!valid_cc_) {
		result_[ValidationError::ValueError].insert("country_code");
	}

	// Check types
	for(auto [key, type] : required_types_) {
		bool failed_validation_{false};
		bool empty_container_{false};
		switch (type) {
			case toml::Type::Integer:
				failed_validation_ = !reader_.get_integer(key).has_value();
				break;
			case toml::Type::String:
				failed_validation_ = !reader_.get_string(key).has_value();
				break;
			case toml::Type::Boolean:
				failed_validation_ = !reader_.get_boolean(key).has_value();
				break;
			case toml::Type::List:
				failed_validation_ = !reader_.get_list(key).has_value();
				if(!failed_validation_) empty_container_ = reader_.get_list(key).value().empty();
				break;
			case toml::Type::Version:
				failed_validation_ = !reader_.get_version(key).has_value();
				break;
			case toml::Type::Date:
				failed_validation_ = !reader_.get_date(key).has_value();
				break;
		}
		if(failed_validation_) {
			result_[ValidationError::IncorrectType].insert(key);
		}
        if(empty_container_) {
			result_[ValidationError::Empty].insert(key);
		}
	}

	for(auto [key, type] : optional_types_) {
		if(!reader_.has_key(key)) continue;
		bool failed_validation_{false};
        bool empty_container_{false};
		switch (type) {
			case toml::Type::Integer:
				failed_validation_ = !reader_.get_integer(key).has_value();
				break;
			case toml::Type::String:
				failed_validation_ = !reader_.get_string(key).has_value();
				break;
			case toml::Type::Boolean:
				failed_validation_ = !reader_.get_boolean(key).has_value();
				break;
			case toml::Type::List:
				failed_validation_ = !reader_.get_list(key).has_value();
				break;
			case toml::Type::Version:
				failed_validation_ = !reader_.get_version(key).has_value();
				break;
			case toml::Type::Date:
				failed_validation_ = !reader_.get_date(key).has_value();
				break;
		}
		if(failed_validation_) {
			result_[ValidationError::IncorrectType].insert(key);
		}
	}

	for(const auto& [key, directory] : file_directories_) {
		if(key == "rly_file" && reader_.get_list(key).has_value()) {
			const std::string rly_file_ = reader_.get_string(key).value();
			const std::filesystem::path file_path_{directory};
			bool file_found_{false};
			for (const auto& entry : std::filesystem::recursive_directory_iterator(file_path_)) {
				if (entry.path().filename() == rly_file_) {
					file_found_ = true;
				}
			}
			if(!file_found_) {
			   result_[ValidationError::FileNotFound].insert(key);
			} else break;
			for (const auto& c : invalid_filename_chars) {
				if(rly_file_.find(c) != std::string::npos) {
					result_[ValidationError::FileNameWarning].insert(key);
					break;
				}
			}
		} else if(reader_.get_list(key).has_value()) {
			const std::vector<std::string> files_ = reader_.get_list(key).value();
			for (const auto& file : files_) {
				const std::filesystem::path file_path_{directory};
				bool file_found_{false};
				for (const auto& entry : std::filesystem::recursive_directory_iterator(file_path_)) {
					if (entry.path().filename() == file) {
						file_found_ = true;
					}
				}
				if(!file_found_) {
				   result_[ValidationError::FileNotFound].insert(key);
				} else break;
				for (const auto& c : invalid_filename_chars) {
					if(file.find(c) != std::string::npos) {
						result_[ValidationError::FileNameWarning].insert(key);
                        break;
					}
				}
			}
        }
    }

	return result_;
}

void ValidationResult::dump(const std::filesystem::path& output_file) const {
	const std::filesystem::path directory_{output_file.parent_path()};
	if(!std::filesystem::exists(directory_)) {
		throw MetadataFileException(
			"Cannot write metadata validation result, directory '" +
			directory_.string() +
			"' directory does not exist."
		);
	}
	std::ofstream output_{output_file};
	output_ << (*this) << std::endl;
	output_.close();
}

std::vector<std::string> MetadataReader::get_list_from_delimited(
	const std::string& value,
	const char delimiter
) {
   	std::vector<std::string> tokens_;
    std::stringstream ss_(value);
	std::string item_;

	while (std::getline(ss_, item_, delimiter)) tokens_.push_back(item_);

	return tokens_;
}

void MetadataReader::read_directory(const std::filesystem::path& directory, bool raise_except) {
	if(!std::filesystem::is_directory(directory)) {
		throw MetadataFileException(
            "Location '" + directory.string() + "' is not a directory."
		);
	}

	for(const auto& entry : std::filesystem::directory_iterator(directory)) {
		 if(std::filesystem::path(entry.path()).extension().string() != ".toml") {
			 continue;
		 }
		 try {
			current_metadata_[entry.path().string()] = read_metadata_from_file(entry.path());
		 } catch(toml::TOMLException& e) {
			 if(raise_except) throw e;
		 } catch(MetadataFileException& e) {
			 if(raise_except) throw e;
         }
    }
}

std::optional<SimulationMetadata> MetadataReader::get_by_prefix(const std::string& prefix) const {
	for(auto [_, metadata] : get_metadata()) {
		if(std::filesystem::path(metadata.rly_file).stem().string().find(prefix) != std::string::npos) {
			return metadata;
		}
		for(const auto ttb_file : metadata.ttb_files) {
			if(std::filesystem::path(ttb_file).stem().string().find(prefix) != std::string::npos) {
				return metadata;
			}
		}
		for(const auto docs_file : metadata.doc_files) {
			if(std::filesystem::path(docs_file).stem().string().find(prefix) != std::string::npos) {
				return metadata;
			}
		}
		if(metadata.ssn_files.has_value()) {
			for(const auto ssn_file : metadata.ssn_files.value()) {
				if(std::filesystem::path(ssn_file).stem().string().find(prefix) != std::string::npos) {
					return metadata;
				}
			}
		}
	}
	return std::nullopt;
}
