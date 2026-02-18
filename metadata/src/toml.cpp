#include "metadata/toml.hxx"
#include <cstdarg>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

using namespace toml;

void Date::validate_() {
	if(month > 12) {
		const std::string err_ = "Value '" + std::to_string(month) +
			"' must be a valid month.";
		throw TOMLException(err_);
	}
	const bool date_too_large_ = day > 31 || (
		(
			(month == 11 || month == 4 || month == 6 || month == 9)  &&
			day > 30
		) ||
		(
			(month == 2)  &&
			day > 29
		) ||
		(
			(month == 1 || month == 3 || month == 5 || month == 7 || month == 10 || month == 12)  &&
			day > 31
		)
	);
	if(date_too_large_) {
		const std::string err_ = "Value '" + std::to_string(day) +
			"' must be a valid day of month " + std::to_string(month) + ".";
		throw TOMLException(err_);
	}
}

void TOML::clear() {
	lists_.clear();
	strings_.clear();
	integers_.clear();
	versions_.clear();
	booleans_.clear();
	dates_.clear();
}

OptionalVector TOML::get_list(const std::string& label) const {
	if(lists_.find(label) == lists_.end()) return std::nullopt;
	const std::vector<std::string> value_ = lists_.at(label);

    return std::make_optional<std::vector<std::string>>(value_);
}

std::optional<int> TOML::get_boolean(const std::string& label) const {
	if(booleans_.find(label) == booleans_.end()) return std::nullopt;
	const int value_ = static_cast<int>(booleans_.at(label));

	return std::make_optional<int>(value_);
}

std::optional<int> TOML::get_integer(const std::string& label) const {
	if(integers_.find(label) == integers_.end()) return std::nullopt;
	const int value_ = integers_.at(label);

	return std::make_optional<int>(value_);
}

std::optional<std::string> TOML::get_string(const std::string& label) const {
	if(strings_.find(label) == strings_.end()) return std::nullopt;
	const std::string value_ = strings_.at(label);

	return std::make_optional<std::string>(value_);
}

std::optional<Date> TOML::get_date(const std::string& label) const {
	if(dates_.find(label) == dates_.end()) return std::nullopt;
	const Date value_ = dates_.at(label);

	return std::make_optional<Date>(value_);
}

std::optional<Version> TOML::get_version(const std::string& label) const {
	if(versions_.find(label) == versions_.end()) return std::nullopt;
	const Version value_ = versions_.at(label);

	return std::make_optional<Version>(value_);
}

void TOML::load(const std::filesystem::path& input_file, bool verbose) {
	clear();

	if (!std::filesystem::exists(input_file)) {
        std::cerr << "Failed to open metadata file " << input_file << std::endl;
        return;
	}

	std::ifstream input_(input_file.string());
	std::string line_;
	std::smatch match_;
	std::string key_;
    std::vector<std::string> list_;

	while(std::getline(input_, line_)) {
		// See if it is a single line list
		if(std::regex_search(line_, match_, parse_key_)) {
			key_ = match_[1];
			list_.clear();
		} else if(std::regex_search(line_, match_, parse_list_element_new_line_)) {
			list_.push_back(match_[1]);
            continue;
		}

		if(std::regex_search(line_, match_, parse_list_start_)) {
			list_.clear();
			continue;
		}

		if(std::regex_search(line_, parse_list_end_)) {
			if(verbose) {
				std::cout << "Found list multiline list: [";
				for(auto element : list_) {
					std::cout << "'" << element << "',";
				}
				std::cout << "]\n";
			}
			lists_[key_] = list_;
        	list_.clear();
		}
		if(std::regex_search(line_, parse_list_same_line_)) {
			list_.clear();
			std::sregex_iterator begin_(line_.begin(), line_.end(), parse_list_element_);
			std::sregex_iterator end_;

			if(begin_ != end_) {
				for(auto it{begin_}; it != end_; ++it) {
					list_.push_back((*it)[1]);
				}
			}
			if(verbose) {
				std::cout << "Found single line list: [";
				for(auto element : list_) {
					std::cout << "'" << element << "',";
				}
				std::cout << "]\n";
			}
			lists_[key_] = list_;
			list_.clear();
		}

		if(std::regex_search(line_, match_, parse_date)) {
			

			if(verbose) std::cout << "Found date " << key_ << "='" << match_[1] << "'" << std::endl;

		   dates_[key_] = Date::from_string(line_);
		   continue;
		}

		if(std::regex_search(line_, match_, parse_version)) {
			if(verbose) std::cout << "Found version " << key_ << "='" << match_[1] << "'" << std::endl;

		   versions_[key_] = Version::from_string(line_);
		   continue;
        }

		if(std::regex_search(line_, match_, parse_integer_)) {
			if(verbose) std::cout << "Found integer " << key_ << "=" << match_[1] << std::endl;
			integers_[key_] = std::stoi(match_[1]);
		}

		if(std::regex_search(line_, match_, parse_boolean_)) {
			if(verbose) std::cout << "Found boolean " << key_ << "=" << match_[1] << std::endl;
			booleans_[key_] = (match_[1] == "true") ? true : false;
		}

        if(std::regex_search(line_, match_, parse_string_)) {
			if(verbose) std::cout << "Found string " << key_ << "='" << match_[1] << "'" << std::endl;
		   strings_[key_] = match_[1];
        }
    }
}

void TOML::dump(const std::filesystem::path& output_file) {
	std::stringstream ss;
	ss << *this;

	std::ofstream output_{output_file};
	output_ << ss.str();
	output_.close();
}
