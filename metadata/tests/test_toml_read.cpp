#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include "metadata/reader.hxx"

#include <filesystem>

#ifndef TEST_DATA_DIRECTORY
#error "TEST_DATA_DIRECTORY must be defined!"
#endif

TEST_CASE("Test Read Example TOML File", "[toml]") {
    std::filesystem::path test_data_file{std::string(TEST_DATA_DIRECTORY)};
    test_data_file = test_data_file / "data" / "Western_Lyon_Tram_Train.toml";
    CHECK(std::filesystem::exists(test_data_file));
    Metadata.read_metadata_from_file(test_data_file);

    CHECK(Metadata.get_metadata().has_value());

    REQUIRE(Metadata.get_metadata().value().name != "Western Lyon Tram Train");
}