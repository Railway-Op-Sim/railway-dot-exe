#pragma once
#include <regex>
#include <map>
#include <vector>
#include <optional>
#include <stdexcept>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <fstream>

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
	const unsigned int major{0};
	const unsigned int minor{0};
    const unsigned int patch{0};
};

class Date {
	private:
        void validate_();
	public:
		const unsigned int year;
		const unsigned int month;
		const unsigned int day;
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
		const std::regex parse_list_start_{R"delim(=\s*\[)delim"};
		const std::regex parse_list_end_{R"delim(\]\s*$)delim"};
		const std::regex parse_integer_{R"delim(=\s*(\d+)\s*$)delim"};
		const std::regex parse_boolean_{R"delim(=\s*(false|true)\s*$)delim"};
        const std::regex parse_string_{R"delim(=\s*"(.*)"\s*$)delim"};
		const std::regex parse_date_{R"delim(=\s*"(\d{4}-\d{2}-\d{2})"\s*$)delim"};
		const std::regex parse_version_{R"delim(=\s*"(\d+\.\d+\.\d+)"\s*$)delim"};
		const std::regex parse_list_element_{R"delim("([^"]*)")delim"};
		const std::regex parse_list_element_new_line_{R"delim(^\s*"(.*)"\s*$)delim"};
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

        // Have to return boolean as integer as C++ Builder hates optional<bool>
		std::optional<int> get_boolean(const std::string& label) const;
		std::optional<int> get_integer(const std::string& label) const;
		void read(const std::filesystem::path& input_file, bool verbose = false);
		void clear();
};