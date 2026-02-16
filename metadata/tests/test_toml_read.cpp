#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "metadata/toml.hxx"
#include "metadata/reader.hxx"

#include <filesystem>
#include <iostream>

#ifndef TEST_DATA_DIRECTORY
#error "TEST_DATA_DIRECTORY must be defined!"
#endif

TEST_CASE("Test Read Example TOML File", "[toml]") {
    std::filesystem::path test_data_file{std::string(TEST_DATA_DIRECTORY)};
    test_data_file = test_data_file / "Western_Lyon_Tram_Train.toml";
    MetadataReader Metadata(std::filesystem::path("."));
    const SessionMetadata metadata_{Metadata.read_metadata_from_file(test_data_file, true)};

    REQUIRE(metadata_.name == "Western Lyon Tram Train");
}

TEST_CASE("Test Read/Write TOML File", "[toml]") {
    std::filesystem::path test_data_file{std::string(TEST_DATA_DIRECTORY)};
    test_data_file = test_data_file / "Western_Lyon_Tram_Train.toml";
    TOML toml_;
    toml_.load(test_data_file);
    const std::string name_{toml_.get<std::string>("name")};
    REQUIRE(name_ == "Western Lyon Tram Train");
    std::filesystem::path out_data_file_{std::string(TEST_DATA_DIRECTORY)};
    out_data_file_ = out_data_file_ / "Test.toml";
    toml_.dump(out_data_file_);
    toml_.clear();
    toml_.load(out_data_file_);
    const std::string name_2_{toml_.get<std::string>("name")};
    REQUIRE(name_2_ == "Western Lyon Tram Train");
}