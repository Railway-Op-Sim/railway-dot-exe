#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
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

    // std::cout << metadata_ << std::endl;

    // REQUIRE(metadata_.name != "Western Lyon Tram Train");
}