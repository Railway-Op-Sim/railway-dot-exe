#pragma once
#include <ostream>
#include <regex>
#include <map>
#include <vector>
#include <optional>
#include <stdexcept>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <fstream>

namespace toml {
	struct Version;
	struct Date;

	using TOMLVectorMap = std::map<std::string, std::vector<std::string>>;
	using TOMLStringMap = std::map<std::string, std::string>;
	using TOMLIntegerMap = std::map<std::string, int>;
	using TOMLDateMap = std::map<std::string, Date>;
	using TOMLSemverMap = std::map<std::string, Version>;
	using TOMLBoolMap = std::map<std::string, bool>;
	using OptionalVector = std::optional<std::vector<std::string>>;

	enum class Type {
		String,
		Integer,
		List,
		Version,
		Date,
		Boolean
	};

	class TOMLException : public std::invalid_argument {
		public:
			explicit TOMLException(const std::string& message) : std::invalid_argument(message) {}
	};

	struct Version {
		unsigned int major{0};
		unsigned int minor{0};
		unsigned int patch{0};
	};

	class Date {
		private:
			void validate_();
		public:
			unsigned int year;
			unsigned int month;
			unsigned int day;
			Date(const unsigned int year, const unsigned int month, const unsigned int day) :
				year(year), month(month), day(day) {
				validate_();
			}
			Date() : year(1973), month(1), day(1) {}
	};

	class TOML {
		private:
			TOMLVectorMap lists_;
			TOMLStringMap strings_;
			TOMLIntegerMap integers_;
			TOMLSemverMap versions_;
			TOMLBoolMap booleans_;
			TOMLDateMap dates_;
			const std::regex parse_key_{R"delim(^(.*)\s=)delim"};
			const std::regex parse_list_start_{R"delim(=\s*\[\s*$)delim"};
			const std::regex parse_list_end_{R"delim(^\]\s*$)delim"};
			const std::regex parse_integer_{R"delim(=\s*(\d+)\s*$)delim"};
			const std::regex parse_boolean_{R"delim(=\s*(false|true)\s*$)delim"};
			const std::regex parse_string_{R"delim(=\s*"(.*)"\s*$)delim"};
			const std::regex parse_date_{R"delim(=\s*"(\d{4}-\d{2}-\d{2})"\s*$)delim"};
			const std::regex parse_version_{R"delim(=\s*"(\d+\.\d+\.\d+)"\s*$)delim"};
			const std::regex parse_list_element_{R"delim("([^"]*)")delim"};
			const std::regex parse_list_element_new_line_{R"delim(^\s*"(.*)")delim"};
			const std::regex parse_list_same_line_{R"delim(=\s*\[(.*)\]\s*$)delim"};
		public:
			OptionalVector get_list(const std::string& label) const;
			std::optional<std::string> get_string(const std::string& label) const;
			std::optional<Date> get_date(const std::string& label) const;
			std::optional<Version> get_version(const std::string& label) const;
			std::optional<Type> has_key(const std::string& label) const {
				if(lists_.find(label) != lists_.end()) return Type::List;
				if(integers_.find(label) != integers_.end()) return Type::Integer;
				if(strings_.find(label) != strings_.end()) return Type::String;
				if(versions_.find(label) != versions_.end()) return Type::Version;
				if(dates_.find(label) != dates_.end()) return Type::Date;
				if(booleans_.find(label) != booleans_.end()) return Type::Boolean;
				return std::nullopt;
			}
			void integer(int value);

			template<typename T>
			void insert(const std::string& key, const T& value) {
				if constexpr (std::is_same_v<T, int>) {
					integers_[key] = value;
				} else if constexpr (std::is_same_v<T, std::string>) {
					strings_[key] = value;
				} else if constexpr (std::is_same_v<T, Date>) {
					dates_[key] = value;
				} else if constexpr (std::is_same_v<T, Version>) {
					versions_[key] = value;
				} else if constexpr (std::is_same_v<T, bool>) {
					booleans_[key] = value;
				} else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
					lists_[key] = value;
				} else {
					throw std::invalid_argument("Unsupported type for value");
				}
			}

			template<typename T>
			T get(const std::string& key) {
				if(!has_key(key)) {
					throw TOMLException(
						"Key '" + key + "' not found."
					);
				}
				if constexpr (std::is_same_v<T, int>) {
					if(integers_.find(key) == integers_.end()) {
						throw TOMLException(
							"Key '" + key + "' not found."
						);
					}
					return integers_[key];
				} else if constexpr (std::is_same_v<T, std::string>) {
					if(strings_.find(key) == strings_.end()) {
						throw TOMLException(
							"Key '" + key + "' not found."
						);
					}
					return strings_[key];
				} else if constexpr (std::is_same_v<T, Date>) {
					if(dates_.find(key) == dates_.end()) {
						throw TOMLException(
							"Key '" + key + "' not found."
						);
					}
					return dates_[key];
				} else if constexpr (std::is_same_v<T, Version>) {
					if(versions_.find(key) == versions_.end()) {
						throw TOMLException(
							"Key '" + key + "' not found."
						);
					}
					return versions_[key];
				} else if constexpr (std::is_same_v<T, bool>) {
					if(booleans_.find(key) == booleans_.end()) {
						throw TOMLException(
							"Key '" + key + "' not found."
						);
					}
					return booleans_[key];
				} else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
					if(lists_.find(key) == lists_.end()) {
						throw TOMLException(
							"Key '" + key + "' not found."
						);
					}
					return lists_[key];
				} else {
					throw std::invalid_argument("Unsupported type for value");
				}
		
			}

			// Have to return boolean as integer as C++ Builder hates optional<bool>
			std::optional<int> get_boolean(const std::string& label) const;
			std::optional<int> get_integer(const std::string& label) const;
			void load(const std::filesystem::path& input_file, bool verbose = false);
			void clear();
			void dump(const std::filesystem::path& output_file);

			friend std::ostream& operator<<(std::ostream& os, const TOML& toml) {
				for(auto [key, date] :toml.dates_) {
					os << key << " = " << '"';
					os << date.year << "-";
					os << ((date.month < 10) ? "0" : "") << date.month << "-";
					os << ((date.day < 10) ? "0" : "") << date.day << '"';
					os << "\n";
				}
				for(auto [key, version] :toml.versions_) {
					os << key << " = " << '"';
					os << version.major << ".";
					os << version.minor << ".";
					os << version.patch << '"';
					os << "\n";
				}
				for(auto [key, string] :toml.strings_) {
					os << key << " = " << '"' << string << '"' << "\n";
				}
				for(auto [key, integer] :toml.integers_) {
					os << key << " = " << integer << "\n";
				}
				for(auto [key, list] :toml.lists_) {
					if(list.empty()) continue;
					os << key << " = [";
					for(auto string : list) {
						os << "\t" << '"' << string << '"' << ",\n";
					}
					os << "]" << "\n";
				}
				for(auto [key, boolean] : toml.booleans_) {
					os << key << " = " << ((boolean) ? "true" : "false") << "\n";
				}
				return os;
			}
	};
};